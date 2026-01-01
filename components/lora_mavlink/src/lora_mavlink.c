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

// Ring buffer structure
typedef struct {
    uint8_t buffer[LORA_TX_BUFFER_SIZE];
    volatile size_t head;
    volatile size_t tail;
    volatile size_t count;
} ring_buffer_t;

// Module state
static struct {
    bool initialized;
    lora_state_t state;
    lora_config_t config;
    lora_stats_t stats;

    ring_buffer_t tx_buffer;
    ring_buffer_t rx_buffer;

    uint8_t tx_packet[LORA_MAX_PACKET_SIZE];
    size_t tx_packet_len;

    SemaphoreHandle_t mutex;
    TaskHandle_t task_handle;

    uint32_t last_tx_time;
    uint32_t tx_interval_ms;
    bool rx_pending;
} lora_ctx = {0};

// Ring buffer operations
static void ring_buffer_init(ring_buffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

static size_t ring_buffer_free(const ring_buffer_t *rb)
{
    return LORA_TX_BUFFER_SIZE - rb->count;
}

static size_t ring_buffer_available(const ring_buffer_t *rb)
{
    return rb->count;
}

static size_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *data, size_t len)
{
    size_t written = 0;
    while (written < len && rb->count < LORA_TX_BUFFER_SIZE) {
        rb->buffer[rb->head] = data[written];
        rb->head = (rb->head + 1) % LORA_TX_BUFFER_SIZE;
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
        rb->tail = (rb->tail + 1) % LORA_TX_BUFFER_SIZE;
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
        tail = (tail + 1) % LORA_TX_BUFFER_SIZE;
        count--;
        peek_count++;
    }
    return peek_count;
}

static void ring_buffer_consume(ring_buffer_t *rb, size_t len)
{
    while (len > 0 && rb->count > 0) {
        rb->tail = (rb->tail + 1) % LORA_TX_BUFFER_SIZE;
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
    int16_t rssi;
    int8_t snr;

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

        ESP_LOGD(TAG, "RX done, len=%d, RSSI=%d, SNR=%d", (int)len, rssi, snr);
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

// Try to start transmission
static bool try_transmit(void)
{
    if (lora_ctx.state != LORA_STATE_IDLE) {
        return false;
    }

    if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }

    // Check if there's data to send
    size_t available = ring_buffer_available(&lora_ctx.tx_buffer);
    if (available == 0) {
        xSemaphoreGive(lora_ctx.mutex);
        return false;
    }

    // Limit packet size
    size_t tx_len = available;
    if (tx_len > LORA_MAX_PACKET_SIZE) {
        tx_len = LORA_MAX_PACKET_SIZE;
    }

    // Read data from TX buffer
    lora_ctx.tx_packet_len = ring_buffer_peek(&lora_ctx.tx_buffer, lora_ctx.tx_packet, tx_len);

    xSemaphoreGive(lora_ctx.mutex);

    if (lora_ctx.tx_packet_len == 0) {
        return false;
    }

    // Start transmission
    if (sx126x_send_packet(lora_ctx.tx_packet, lora_ctx.tx_packet_len)) {
        lora_ctx.state = LORA_STATE_TX;

        // Consume data from TX buffer
        if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ring_buffer_consume(&lora_ctx.tx_buffer, lora_ctx.tx_packet_len);
            xSemaphoreGive(lora_ctx.mutex);
        }

        ESP_LOGD(TAG, "TX started, len=%d", (int)lora_ctx.tx_packet_len);
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
        }
    }
}

// LoRa task function
static void lora_task(void *arg)
{
    ESP_LOGI(TAG, "LoRa task started");

    while (1) {
        // Check for DIO1 interrupt
        if (sx126x_hal_get_dio1()) {
            process_irq();
        }

        // State machine
        switch (lora_ctx.state) {
            case LORA_STATE_IDLE:
                // Try to transmit if there's data
                if (!try_transmit()) {
                    // No TX, go to RX
                    start_rx();
                }
                break;

            case LORA_STATE_RX:
                // Check for incoming data periodically
                // If TX data is pending for too long, do CAD and transmit
                if (ring_buffer_available(&lora_ctx.tx_buffer) > 0) {
                    uint32_t elapsed = get_time_ms() - lora_ctx.last_tx_time;
                    if (elapsed > lora_ctx.tx_interval_ms) {
                        // Return to idle to allow TX
                        sx126x_set_standby(0);
                        lora_ctx.state = LORA_STATE_IDLE;
                    }
                }
                break;

            case LORA_STATE_TX:
                // Wait for TX done (handled by IRQ)
                break;

            case LORA_STATE_CAD:
                // Wait for CAD done (handled by IRQ)
                break;

            case LORA_STATE_ERROR:
                // Try to recover
                ESP_LOGE(TAG, "Error state, reinitializing");
                sx126x_set_standby(0);
                vTaskDelay(pdMS_TO_TICKS(100));
                lora_ctx.state = LORA_STATE_IDLE;
                break;
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
    ring_buffer_init(&lora_ctx.tx_buffer);
    ring_buffer_init(&lora_ctx.rx_buffer);

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

    // Set initial state
    lora_ctx.state = LORA_STATE_IDLE;
    lora_ctx.last_tx_time = get_time_ms();
    lora_ctx.tx_interval_ms = 100;  // Minimum TX interval

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

size_t lora_mavlink_write(const uint8_t *data, size_t len)
{
    if (!lora_ctx.initialized || !data || len == 0) {
        return 0;
    }

    size_t written = 0;

    if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        written = ring_buffer_write(&lora_ctx.tx_buffer, data, len);
        xSemaphoreGive(lora_ctx.mutex);
    }

    return written;
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
        free_space = ring_buffer_free(&lora_ctx.tx_buffer);
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
