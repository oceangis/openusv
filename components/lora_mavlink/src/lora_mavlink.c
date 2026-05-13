/**
 * @file lora_mavlink.c
 * @brief LoRa MAVLink communication interface implementation for ESP32-S3
 *
 * Provides LoRa-based MAVLink telemetry using SX1262 radio module.
 */

#include "lora_mavlink.h"
#include "sx126x_hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <string.h>

static const char *TAG = "lora_mavlink";

// External SX126x driver functions
extern bool sx126x_init(const lora_config_t *config);
extern void sx126x_deinit(void);
extern bool sx126x_is_initialized(void);
extern bool sx126x_send_packet(const uint8_t *data, size_t len);
extern bool sx126x_start_rx_continuous(void);
extern size_t sx126x_receive_packet(uint8_t *data, size_t max_len, int16_t *rssi, int8_t *snr);
extern bool sx126x_start_cad(void);
extern void sx126x_set_standby(uint8_t mode);
extern void sx126x_set_rf_frequency(uint32_t freq_hz);
extern void sx126x_set_pa_config(int8_t power_dbm);
extern void sx126x_set_tx_params(int8_t power_dbm, uint8_t ramp_time);
extern uint16_t sx126x_get_irq_status(void);
extern void sx126x_clear_irq_status(uint16_t irq_mask);
extern uint8_t sx126x_get_status(void);

// Ring buffer structure (generic, size passed to operations)
typedef struct {
    uint8_t *buffer;
    size_t   buf_size;
    volatile size_t head;
    volatile size_t tail;
    volatile size_t count;
} ring_buffer_t;

// MAVLink v2 parser state for TX priority routing
typedef enum {
    MAV_PARSE_IDLE,       // Waiting for 0xFD
    MAV_PARSE_LEN,        // Got magic, reading payload length
    MAV_PARSE_HEADER,     // Reading header bytes (incompat, compat, seq, sysid, compid)
    MAV_PARSE_MSGID,      // Reading message ID (3 bytes for v2)
    MAV_PARSE_PAYLOAD,    // Reading payload + CRC
} mav_parse_state_t;

typedef struct {
    mav_parse_state_t state;
    uint8_t  pkt_buf[280];     // Max MAVLink v2 packet
    size_t   pkt_idx;
    uint8_t  payload_len;
    uint32_t msg_id;
    size_t   expected_len;     // Total packet length
} mav_parser_t;

// High-priority MAVLink message IDs (command responses, mission protocol, essential)
static bool is_high_priority_msg(uint32_t msg_id)
{
    switch (msg_id) {
        case 0:    // HEARTBEAT
        case 22:   // PARAM_VALUE
        // Mission protocol (both old and INT variants)
        case 39:   // MISSION_ITEM (old protocol, GCS→vehicle)
        case 40:   // MISSION_REQUEST (old protocol, vehicle→GCS) ← was missing!
        case 43:   // MISSION_REQUEST_LIST (vehicle→GCS, mission download)
        case 44:   // MISSION_COUNT
        case 47:   // MISSION_ACK
        case 51:   // MISSION_REQUEST_INT (new protocol, vehicle→GCS)
        case 73:   // MISSION_ITEM_INT (new protocol, GCS→vehicle)
        case 77:   // COMMAND_ACK
            return true;
        default:
            return false;
    }
}

// LoRa telemetry filter: minimal messages for remote controller display
// Not a ground station — only what the RC screen needs
static bool is_allowed_telemetry_msg(uint32_t msg_id)
{
    switch (msg_id) {
        // High-priority (command/param responses)
        case 0:    // HEARTBEAT — mode, armed status
        case 22:   // PARAM_VALUE — param read response
        case 77:   // COMMAND_ACK — command response
        // Essential telemetry for RC display
        case 33:   // GLOBAL_POSITION_INT — GPS position (36B)
        case 74:   // VFR_HUD — heading, speed, throttle (24B)
            return true;
        default:
            return false;
    }
}

// Static buffer storage
static uint8_t tx_lo_buf_storage[LORA_TX_BUFFER_SIZE];
static uint8_t tx_hi_buf_storage[LORA_TX_HI_BUFFER_SIZE];
static uint8_t rx_buf_storage[LORA_RX_BUFFER_SIZE];

// Module state
static struct {
    bool initialized;
    lora_state_t state;
    lora_config_t config;
    lora_stats_t stats;

    ring_buffer_t tx_lo;       // Low-priority TX (telemetry)
    ring_buffer_t tx_hi;       // High-priority TX (command responses)
    ring_buffer_t rx_buffer;

    mav_parser_t parser;       // MAVLink TX parser for priority routing

    uint8_t tx_packet[LORA_MAX_PACKET_SIZE];
    size_t tx_packet_len;

    SemaphoreHandle_t mutex;
    TaskHandle_t task_handle;

    uint32_t last_tx_time;

    // RX-dominant scheduling: default RX, periodic telemetry TX
    uint32_t last_telem_tx_time;    // Last time we sent a telemetry packet
} lora_ctx = {0};

// Ring buffer operations
static void ring_buffer_init(ring_buffer_t *rb, uint8_t *storage, size_t size)
{
    rb->buffer = storage;
    rb->buf_size = size;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

static size_t ring_buffer_free(const ring_buffer_t *rb)
{
    return rb->buf_size - rb->count;
}

static size_t ring_buffer_available(const ring_buffer_t *rb)
{
    return rb->count;
}

static size_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *data, size_t len)
{
    size_t written = 0;
    while (written < len && rb->count < rb->buf_size) {
        rb->buffer[rb->head] = data[written];
        rb->head = (rb->head + 1) % rb->buf_size;
        rb->count++;
        written++;
    }
    return written;
}

static size_t ring_buffer_read(ring_buffer_t *rb, uint8_t *data, size_t len)
{
    size_t read_count = 0;
    while (read_count < len && rb->count > 0) {
        data[read_count] = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % rb->buf_size;
        rb->count--;
        read_count++;
    }
    return read_count;
}

static size_t ring_buffer_peek(const ring_buffer_t *rb, uint8_t *data, size_t len)
{
    size_t peek_count = 0;
    size_t tail = rb->tail;
    size_t count = rb->count;

    while (peek_count < len && count > 0) {
        data[peek_count] = rb->buffer[tail];
        tail = (tail + 1) % rb->buf_size;
        count--;
        peek_count++;
    }
    return peek_count;
}

static void ring_buffer_consume(ring_buffer_t *rb, size_t len)
{
    while (len > 0 && rb->count > 0) {
        rb->tail = (rb->tail + 1) % rb->buf_size;
        rb->count--;
        len--;
    }
}

// Get current time in ms
static uint32_t get_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// Handle TX complete
static void handle_tx_done(void)
{
    lora_ctx.stats.tx_packets++;
    lora_ctx.stats.tx_bytes += lora_ctx.tx_packet_len;
    lora_ctx.tx_packet_len = 0;
    lora_ctx.last_tx_time = get_time_ms();
    lora_ctx.state = LORA_STATE_IDLE;

    ESP_LOGD(TAG, "TX done, packets=%lu", (unsigned long)lora_ctx.stats.tx_packets);
}

// Handle RX complete
static void handle_rx_done(void)
{
    uint8_t rx_data[LORA_MAX_PACKET_SIZE];
    int16_t rssi = 0;
    int8_t snr = 0;

    size_t len = sx126x_receive_packet(rx_data, sizeof(rx_data), &rssi, &snr);

    if (len > 0) {
        lora_ctx.stats.rx_packets++;
        lora_ctx.stats.rx_bytes += len;
        lora_ctx.stats.last_rssi = rssi;
        lora_ctx.stats.last_snr = snr;

        // Write to RX ring buffer
        if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            size_t written = ring_buffer_write(&lora_ctx.rx_buffer, rx_data, len);
            if (written < len) {
                ESP_LOGW(TAG, "RX buffer overflow, lost %d bytes", (int)(len - written));
            }
            xSemaphoreGive(lora_ctx.mutex);
        }

        ESP_LOGI(TAG, "RX done, len=%d, RSSI=%d, SNR=%d", (int)len, rssi, snr);
    }

    lora_ctx.state = LORA_STATE_IDLE;
}

// Handle CRC error
static void handle_crc_error(void)
{
    lora_ctx.stats.crc_errors++;
    lora_ctx.state = LORA_STATE_IDLE;
    ESP_LOGW(TAG, "CRC error, total=%lu", (unsigned long)lora_ctx.stats.crc_errors);
}

// Handle timeout
static void handle_timeout(void)
{
    lora_ctx.state = LORA_STATE_IDLE;
    ESP_LOGD(TAG, "RX/TX timeout");
}

// Handle CAD done
static void handle_cad_done(bool activity_detected)
{
    if (activity_detected) {
        // Channel is busy, wait before transmitting
        lora_ctx.state = LORA_STATE_RX;
        sx126x_start_rx_continuous();
        ESP_LOGD(TAG, "CAD: activity detected, switching to RX");
    } else {
        // Channel is free, can transmit
        lora_ctx.state = LORA_STATE_IDLE;
        ESP_LOGD(TAG, "CAD: channel clear");
    }
}

// Process IRQ events
static void process_irq(void)
{
    uint16_t irq = sx126x_get_irq_status();
    if (irq == 0) {
        return;
    }

    ESP_LOGI(TAG, "IRQ detected: 0x%04X (state=%d)", irq, lora_ctx.state);

    // Clear IRQ flags
    sx126x_clear_irq_status(irq);

    // Process events
    if (irq & SX126X_IRQ_TX_DONE) {
        handle_tx_done();
    }

    if (irq & SX126X_IRQ_RX_DONE) {
        if (irq & SX126X_IRQ_CRC_ERR) {
            handle_crc_error();
        } else {
            handle_rx_done();
        }
    } else if (irq & SX126X_IRQ_CRC_ERR) {
        handle_crc_error();
    }

    if (irq & SX126X_IRQ_RX_TX_TIMEOUT) {
        handle_timeout();
    }

    if (irq & SX126X_IRQ_CAD_DONE) {
        bool activity = (irq & SX126X_IRQ_CAD_ACTIVITY_DETECTED) != 0;
        handle_cad_done(activity);
    }
}

// Try to start transmission (high-priority first)
static bool try_transmit(void)
{
    if (lora_ctx.state != LORA_STATE_IDLE) {
        return false;
    }

    if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }

    // Pick buffer: high-priority first, then low-priority
    ring_buffer_t *src = NULL;
    if (ring_buffer_available(&lora_ctx.tx_hi) > 0) {
        src = &lora_ctx.tx_hi;
    } else if (ring_buffer_available(&lora_ctx.tx_lo) > 0) {
        src = &lora_ctx.tx_lo;
    }

    if (src == NULL) {
        xSemaphoreGive(lora_ctx.mutex);
        return false;
    }

    // Limit packet size to reduce air time and allow more RX windows
    size_t available = ring_buffer_available(src);
    size_t max_tx = (src == &lora_ctx.tx_hi) ? LORA_MAX_PACKET_SIZE : LORA_TX_MAX_BYTES;
    size_t peek_len = available;
    if (peek_len > max_tx) {
        peek_len = max_tx;
    }

    // Read data from selected TX buffer
    size_t peeked = ring_buffer_peek(src, lora_ctx.tx_packet, peek_len);

    // Ensure we only send complete MAVLink v2 messages (don't split across LoRa packets)
    // Scan for message boundaries: each message starts with 0xFD
    size_t tx_len = 0;
    size_t pos = 0;
    while (pos < peeked) {
        if (lora_ctx.tx_packet[pos] != 0xFD) {
            pos++;  // Skip garbage
            tx_len = pos;  // Still include skipped bytes to consume them
            continue;
        }
        if (pos + 2 > peeked) break;  // Not enough data for length byte
        uint8_t payload_len = lora_ctx.tx_packet[pos + 1];
        size_t msg_len = 12 + payload_len;  // 10 header + payload + 2 CRC
        // Check signing flag (incompat flags at pos+2)
        if (pos + 3 <= peeked && (lora_ctx.tx_packet[pos + 2] & 0x01)) {
            msg_len += 13;  // Signature
        }
        if (pos + msg_len > peeked) break;  // Incomplete message, don't include
        pos += msg_len;
        tx_len = pos;  // Include this complete message
    }

    lora_ctx.tx_packet_len = tx_len;

    xSemaphoreGive(lora_ctx.mutex);

    if (lora_ctx.tx_packet_len == 0) {
        return false;
    }

    // Start transmission
    if (sx126x_send_packet(lora_ctx.tx_packet, lora_ctx.tx_packet_len)) {
        lora_ctx.state = LORA_STATE_TX;

        // Consume data from the source buffer
        if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ring_buffer_consume(src, lora_ctx.tx_packet_len);
            xSemaphoreGive(lora_ctx.mutex);
        }

        ESP_LOGD(TAG, "TX started, len=%d, hi=%d", (int)lora_ctx.tx_packet_len,
                 (src == &lora_ctx.tx_hi) ? 1 : 0);
        return true;
    }

    return false;
}

// Start receiving
static void start_rx(void)
{
    if (lora_ctx.state == LORA_STATE_IDLE) {
        if (sx126x_start_rx_continuous()) {
            lora_ctx.state = LORA_STATE_RX;
            // Read back chip status to verify RX mode
            uint8_t status = sx126x_get_status();
            uint8_t chip_mode = (status >> 4) & 0x07;
            ESP_LOGI(TAG, "Entered RX mode (status=0x%02X, chipMode=%d, DIO1=%d)",
                     status, chip_mode, sx126x_hal_get_dio1() ? 1 : 0);
        } else {
            ESP_LOGW(TAG, "Failed to start RX");
        }
    }
}

// Check if it's time to send a telemetry packet
static bool should_send_telemetry(void)
{
    uint32_t now = get_time_ms();
    return (now - lora_ctx.last_telem_tx_time >= LORA_TELEM_INTERVAL_MS);
}

// LoRa task function
static void lora_task(void *arg)
{
    ESP_LOGI(TAG, "LoRa task started");
    uint32_t last_stats_time = get_time_ms();
    uint32_t tx_start_time = 0;  // Track when TX started for timeout

    while (1) {
        // Always poll IRQ status register directly (DIO1 pulse too short for 1ms polling)
        process_irq();

        // State machine: RX-dominant
        // Default: stay in RX to receive commands/throttle
        // High-priority TX: send immediately (command responses, mission protocol)
        // Low-priority TX: send periodically (telemetry)
        bool has_hi = (ring_buffer_available(&lora_ctx.tx_hi) > 0);
        bool has_lo = (ring_buffer_available(&lora_ctx.tx_lo) > 0);
        bool should_tx = has_hi || (should_send_telemetry() && has_lo);

        switch (lora_ctx.state) {
            case LORA_STATE_IDLE:
                if (should_tx) {
                    if (try_transmit()) {
                        tx_start_time = get_time_ms();
                        if (!has_hi) {
                            lora_ctx.last_telem_tx_time = get_time_ms();
                        }
                    } else {
                        start_rx();
                    }
                } else {
                    start_rx();
                }
                break;

            case LORA_STATE_RX:
                // Leave RX for high-priority TX immediately, or telemetry on schedule
                if (should_tx) {
                    sx126x_set_standby(0);
                    lora_ctx.state = LORA_STATE_IDLE;
                }
                break;

            case LORA_STATE_TX: {
                // TX timeout: SF7/50B should complete in <20ms, 500ms is very generous
                uint32_t tx_elapsed = get_time_ms() - tx_start_time;
                if (tx_elapsed > 500) {
                    ESP_LOGW(TAG, "TX timeout after %lums, forcing IDLE", (unsigned long)tx_elapsed);
                    sx126x_set_standby(0);
                    sx126x_clear_irq_status(0xFFFF);
                    lora_ctx.state = LORA_STATE_IDLE;
                }
                break;
            }

            case LORA_STATE_CAD:
                break;

            case LORA_STATE_ERROR:
                ESP_LOGE(TAG, "Error state, reinitializing");
                sx126x_set_standby(0);
                vTaskDelay(pdMS_TO_TICKS(100));
                lora_ctx.state = LORA_STATE_IDLE;
                break;
        }

        // Periodic stats and diagnostic logging (every 5s)
        uint32_t now = get_time_ms();
        if (now - last_stats_time >= 5000) {
            // Read SX1268 status and IRQ directly (bypass DIO1 check)
            uint16_t irq_raw = sx126x_get_irq_status();
            uint8_t chip_status = sx126x_get_status();
            uint8_t chip_mode = (chip_status >> 4) & 0x07;
            int dio1_level = sx126x_hal_get_dio1() ? 1 : 0;

            ESP_LOGI(TAG, "Stats: tx=%lu rx=%lu crc=%lu st=%d cm=%d irq=0x%04X dio1=%d txhi=%d txlo=%d rxbuf=%d",
                     (unsigned long)lora_ctx.stats.tx_packets,
                     (unsigned long)lora_ctx.stats.rx_packets,
                     (unsigned long)lora_ctx.stats.crc_errors,
                     lora_ctx.state, chip_mode, irq_raw, dio1_level,
                     (int)ring_buffer_available(&lora_ctx.tx_hi),
                     (int)ring_buffer_available(&lora_ctx.tx_lo),
                     (int)ring_buffer_available(&lora_ctx.rx_buffer));
            last_stats_time = now;
        }

        // Small delay to prevent tight loop
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool lora_mavlink_init(const lora_config_t *config)
{
    if (lora_ctx.initialized) {
        return true;
    }

    ESP_LOGI(TAG, "Initializing LoRa MAVLink interface");

    // Set default config - matched with E22-400MBL-SC evaluation kit
    if (config) {
        memcpy(&lora_ctx.config, config, sizeof(lora_config_t));
    } else {
        lora_ctx.config.frequency = LORA_FREQ_HZ;       // 433 MHz
        lora_ctx.config.tx_power = LORA_TX_POWER;       // 22 dBm
        lora_ctx.config.spreading_factor = LORA_SF;     // SF11
        // Bandwidth: 0=125kHz, 1=250kHz, 2=500kHz
        #if LORA_BW == 500
        lora_ctx.config.bandwidth = 2;  // 500kHz - E22-400MBL default
        #elif LORA_BW == 250
        lora_ctx.config.bandwidth = 1;  // 250kHz
        #else
        lora_ctx.config.bandwidth = 0;  // 125kHz
        #endif
        lora_ctx.config.coding_rate = 1;  // 4/5 (CR4/5)
        lora_ctx.config.preamble_len = 8;
        lora_ctx.config.crc_enable = true;
    }

    // Initialize ring buffers
    ring_buffer_init(&lora_ctx.tx_lo, tx_lo_buf_storage, LORA_TX_BUFFER_SIZE);
    ring_buffer_init(&lora_ctx.tx_hi, tx_hi_buf_storage, LORA_TX_HI_BUFFER_SIZE);
    ring_buffer_init(&lora_ctx.rx_buffer, rx_buf_storage, LORA_RX_BUFFER_SIZE);

    // Initialize MAVLink parser
    memset(&lora_ctx.parser, 0, sizeof(mav_parser_t));

    // Reset statistics
    memset(&lora_ctx.stats, 0, sizeof(lora_stats_t));

    // Create mutex
    lora_ctx.mutex = xSemaphoreCreateMutex();
    if (lora_ctx.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }

    // Initialize SX126x
    if (!sx126x_init(&lora_ctx.config)) {
        ESP_LOGE(TAG, "Failed to initialize SX126x");
        vSemaphoreDelete(lora_ctx.mutex);
        lora_ctx.mutex = NULL;
        return false;
    }

    // Set initial state: RX-dominant, start in IDLE (will enter RX immediately)
    lora_ctx.state = LORA_STATE_IDLE;
    lora_ctx.last_tx_time = get_time_ms();
    lora_ctx.last_telem_tx_time = get_time_ms();

    // Create LoRa task
    BaseType_t ret = xTaskCreate(lora_task, "lora_task", 4096, NULL, 5, &lora_ctx.task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LoRa task");
        sx126x_deinit();
        vSemaphoreDelete(lora_ctx.mutex);
        lora_ctx.mutex = NULL;
        return false;
    }

    lora_ctx.initialized = true;
    ESP_LOGI(TAG, "LoRa MAVLink interface initialized, freq=%lu Hz, power=%d dBm, SF=%d",
             (unsigned long)lora_ctx.config.frequency,
             lora_ctx.config.tx_power,
             lora_ctx.config.spreading_factor);

    return true;
}

void lora_mavlink_deinit(void)
{
    if (!lora_ctx.initialized) {
        return;
    }

    ESP_LOGI(TAG, "Deinitializing LoRa MAVLink interface");

    // Stop task
    if (lora_ctx.task_handle) {
        vTaskDelete(lora_ctx.task_handle);
        lora_ctx.task_handle = NULL;
    }

    // Deinitialize SX126x
    sx126x_deinit();

    // Delete mutex
    if (lora_ctx.mutex) {
        vSemaphoreDelete(lora_ctx.mutex);
        lora_ctx.mutex = NULL;
    }

    lora_ctx.initialized = false;
    lora_ctx.state = LORA_STATE_IDLE;
}

// Route a complete MAVLink packet to the appropriate TX buffer
static void route_mavlink_packet(mav_parser_t *p)
{
    ring_buffer_t *dst;

    if (is_high_priority_msg(p->msg_id)) {
        // High-priority: always pass (command responses, mission protocol)
        dst = &lora_ctx.tx_hi;
    } else if (!is_allowed_telemetry_msg(p->msg_id)) {
        // Low-priority and not in allowed list: drop
        return;
    } else {
        dst = &lora_ctx.tx_lo;
        // Drop this message if buffer full (don't corrupt existing data)
        if (ring_buffer_free(dst) < p->pkt_idx) {
            return;
        }
    }

    // Log mission protocol messages for debugging
    if (p->msg_id >= 39 && p->msg_id <= 51) {
        ESP_LOGI(TAG, "TX route: mission msg_id=%lu len=%d -> %s",
                 (unsigned long)p->msg_id, (int)p->pkt_idx,
                 (dst == &lora_ctx.tx_hi) ? "HI" : "LO");
    }

    ring_buffer_write(dst, p->pkt_buf, p->pkt_idx);
}

size_t lora_mavlink_write(const uint8_t *data, size_t len)
{
    if (!lora_ctx.initialized || !data || len == 0) {
        return 0;
    }

    if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return 0;
    }

    mav_parser_t *p = &lora_ctx.parser;

    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        switch (p->state) {
            case MAV_PARSE_IDLE:
                if (byte == 0xFD) {  // MAVLink v2 magic
                    p->pkt_buf[0] = byte;
                    p->pkt_idx = 1;
                    p->state = MAV_PARSE_LEN;
                }
                // Ignore MAVLink v1 (0xFE) and garbage
                break;

            case MAV_PARSE_LEN:
                p->payload_len = byte;
                p->pkt_buf[p->pkt_idx++] = byte;
                // MAVLink v2: 10 header + payload + 2 CRC (+ optional 13 signing)
                p->expected_len = 12 + p->payload_len;
                p->state = MAV_PARSE_HEADER;
                break;

            case MAV_PARSE_HEADER:
                p->pkt_buf[p->pkt_idx++] = byte;
                // After header bytes (incompat, compat, seq, sysid, compid = 5 bytes)
                // pkt_idx will be 7 when we've got all header bytes
                if (p->pkt_idx == 7) {
                    p->state = MAV_PARSE_MSGID;
                    p->msg_id = 0;
                }
                break;

            case MAV_PARSE_MSGID:
                p->pkt_buf[p->pkt_idx++] = byte;
                // Message ID is 3 bytes (indices 7, 8, 9), little-endian
                if (p->pkt_idx == 8) {
                    p->msg_id = byte;
                } else if (p->pkt_idx == 9) {
                    p->msg_id |= (uint32_t)byte << 8;
                } else if (p->pkt_idx == 10) {
                    p->msg_id |= (uint32_t)byte << 16;
                    // Check if signing flag is set (incompat flags byte at index 2)
                    if (p->pkt_buf[2] & 0x01) {
                        p->expected_len += 13;  // Signature
                    }
                    p->state = MAV_PARSE_PAYLOAD;
                }
                break;

            case MAV_PARSE_PAYLOAD:
                if (p->pkt_idx < sizeof(p->pkt_buf)) {
                    p->pkt_buf[p->pkt_idx++] = byte;
                }
                if (p->pkt_idx >= p->expected_len) {
                    // Complete packet - route to appropriate buffer
                    route_mavlink_packet(p);
                    p->state = MAV_PARSE_IDLE;
                }
                break;
        }

        // Safety: reset if packet gets too long
        if (p->pkt_idx >= sizeof(p->pkt_buf)) {
            p->state = MAV_PARSE_IDLE;
        }
    }

    xSemaphoreGive(lora_ctx.mutex);

    return len;  // Always accept all bytes from ArduPilot
}

size_t lora_mavlink_read(uint8_t *data, size_t len)
{
    if (!lora_ctx.initialized || !data || len == 0) {
        return 0;
    }

    size_t read_count = 0;

    if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        read_count = ring_buffer_read(&lora_ctx.rx_buffer, data, len);
        xSemaphoreGive(lora_ctx.mutex);
    }

    return read_count;
}

size_t lora_mavlink_available(void)
{
    if (!lora_ctx.initialized) {
        return 0;
    }

    size_t available = 0;

    if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        available = ring_buffer_available(&lora_ctx.rx_buffer);
        xSemaphoreGive(lora_ctx.mutex);
    }

    return available;
}

size_t lora_mavlink_tx_free(void)
{
    if (!lora_ctx.initialized) {
        return 0;
    }

    size_t free_space = 0;

    if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Report combined free space (ArduPilot uses this to decide if it can write)
        free_space = ring_buffer_free(&lora_ctx.tx_lo) + ring_buffer_free(&lora_ctx.tx_hi);
        xSemaphoreGive(lora_ctx.mutex);
    }

    return free_space;
}

void lora_mavlink_process(void)
{
    // Processing is done in the dedicated task
    // This function is provided for polling mode if needed
    if (!lora_ctx.initialized) {
        return;
    }

    // Check for DIO1 interrupt
    if (sx126x_hal_get_dio1()) {
        process_irq();
    }
}

lora_state_t lora_mavlink_get_state(void)
{
    return lora_ctx.state;
}

void lora_mavlink_get_stats(lora_stats_t *stats)
{
    if (!stats) {
        return;
    }

    if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(stats, &lora_ctx.stats, sizeof(lora_stats_t));
        xSemaphoreGive(lora_ctx.mutex);
    }
}

bool lora_mavlink_set_frequency(uint32_t freq_hz)
{
    if (!lora_ctx.initialized) {
        return false;
    }

    // Check frequency range (400-500 MHz or 800-930 MHz typical for SX1262)
    if (freq_hz < 150000000 || freq_hz > 960000000) {
        return false;
    }

    // Set standby and change frequency
    sx126x_set_standby(0);
    sx126x_set_rf_frequency(freq_hz);
    lora_ctx.config.frequency = freq_hz;

    // Return to previous state
    if (lora_ctx.state == LORA_STATE_RX) {
        sx126x_start_rx_continuous();
    }

    ESP_LOGI(TAG, "Frequency changed to %lu Hz", (unsigned long)freq_hz);
    return true;
}

bool lora_mavlink_set_tx_power(int8_t power_dbm)
{
    if (!lora_ctx.initialized) {
        return false;
    }

    // Clamp to valid range for SX1262
    if (power_dbm < -9) power_dbm = -9;
    if (power_dbm > 22) power_dbm = 22;

    sx126x_set_pa_config(power_dbm);
    sx126x_set_tx_params(power_dbm, 0x04);  // 200us ramp
    lora_ctx.config.tx_power = power_dbm;

    ESP_LOGI(TAG, "TX power changed to %d dBm", power_dbm);
    return true;
}

bool lora_mavlink_is_initialized(void)
{
    return lora_ctx.initialized;
}
