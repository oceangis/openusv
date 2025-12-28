/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Support for BNO08x IMU with sensor fusion via I2C
  ESP32-S3 specific implementation - v2.1 FIXED

  This version includes critical fixes based on deep analysis of the Adafruit library:

  CRITICAL FIXES in v2.1:
  1. REMOVED INT pin polling for I2C mode (INT is ONLY for SPI in Adafruit!)
  2. FIXED I2C chunked read logic - first read has NO header, subsequent reads have header
  3. Changed to continuous polling with 5ms delay (matching Arduino test code)
  4. Improved debug logging to trace packet flow

  Key implementation details:
  1. 500ms startup delay before first communication (CRITICAL for BNO08x boot)
  2. Soft reset uses exact packet format from Adafruit: {5, 0, 1, 0, 1}
  3. Multiple retry mechanism (up to 5 attempts) matching Arduino code
  4. I2C read NOW matches Adafruit i2chal_read() EXACTLY
  5. wasReset() detection and automatic reconfiguration
  6. Proper SHTP sequence number handling

  Reference: Adafruit_BNO08x.cpp lines 286-387 (i2chal_open and i2chal_read)
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_BNO08X_ENABLED

#include "AP_ExternalAHRS_BNO08x.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

// ESP-IDF GPIO for INT pin
#include "driver/gpio.h"

extern const AP_HAL::HAL &hal;

// ============================================================================
// Constructor
// ============================================================================

AP_ExternalAHRS_BNO08x::AP_ExternalAHRS_BNO08x(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state)
    : AP_ExternalAHRS_backend(frontend, state)
    , thread_started(false)
    , rx_packet_len(0)
    , rx_channel(0)
    , rx_sequence(0)
    , reset_occurred(false)
    , sensor_initialized(false)
    , last_packet_ms(0)
    , last_rotation_ms(0)
    , last_gyro_ms(0)
    , last_accel_ms(0)
    , quat_accuracy_rad(0)
    , quat_status(0)
    , have_rotation(false)
    , have_gyro_data(false)
    , have_accel_data(false)
    , packet_count(0)
    , error_count(0)
    , reset_count(0)
{
    // Initialize sequence numbers for all channels
    memset(seq_num, 0, sizeof(seq_num));

    // Initialize product ID
    memset(&product_id, 0, sizeof(product_id));

    // Create update thread - this is CRITICAL to avoid blocking main loop
    // All I2C operations happen in this thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_BNO08x::update_thread, void),
                                       "BNO08x", 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Failed to create thread");
    }
}

// ============================================================================
// Utility Functions (from Hillcrest sh2_util)
// ============================================================================

float AP_ExternalAHRS_BNO08x::q_to_float(int16_t fixed_point, uint8_t q_point)
{
    return (float)fixed_point / (float)(1 << q_point);
}

int16_t AP_ExternalAHRS_BNO08x::read_i16_le(const uint8_t *buffer)
{
    return (int16_t)(buffer[0] | (buffer[1] << 8));
}

uint16_t AP_ExternalAHRS_BNO08x::read_u16_le(const uint8_t *buffer)
{
    return (uint16_t)(buffer[0] | (buffer[1] << 8));
}

uint32_t AP_ExternalAHRS_BNO08x::read_u32_le(const uint8_t *buffer)
{
    return (uint32_t)(buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24));
}

// ============================================================================
// INT Pin Handling
// ============================================================================

bool AP_ExternalAHRS_BNO08x::init_int_pin()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BNO08X_INT_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Pull-up, INT is active low

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: INT pin config failed");
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: INT pin GPIO%d configured", BNO08X_INT_PIN);
    return true;
}

bool AP_ExternalAHRS_BNO08x::data_ready()
{
    // INT is active low - when low, data is available
    return gpio_get_level((gpio_num_t)BNO08X_INT_PIN) == 0;
}

bool AP_ExternalAHRS_BNO08x::wait_for_int(uint32_t timeout_ms)
{
    uint32_t start = AP_HAL::millis();
    while (AP_HAL::millis() - start < timeout_ms) {
        if (data_ready()) {
            return true;
        }
        hal.scheduler->delay(1);
    }
    return false;
}

// ============================================================================
// Low-Level I2C Functions - EXACT match with Adafruit i2chal_read/write
// ============================================================================

/**
 * hal_read - Read data from BNO08x
 *
 * This EXACTLY matches Adafruit i2chal_read() in Adafruit_BNO08x.cpp lines 307-386
 *
 * Key points:
 * 1. First read 4-byte header to get packet size
 * 2. If packet size > I2C buffer, read in chunks
 * 3. Each subsequent read includes 4-byte header that must be skipped
 *
 * @param pBuffer Output buffer
 * @param len Maximum buffer length
 * @return Number of bytes read, or 0 on error
 */
int AP_ExternalAHRS_BNO08x::hal_read(uint8_t *pBuffer, unsigned len)
{
    if (!dev) {
        return 0;
    }

    // Step 1: Read 4-byte header first
    uint8_t header[4];
    if (!dev->transfer(nullptr, 0, header, 4)) {
        return 0;
    }

    // Parse packet size from header
    uint16_t packet_size = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
    // Clear the continuation bit
    packet_size &= ~0x8000;

    // Check for invalid packet
    if (packet_size == 0 || packet_size == 0x7FFF) {
        return 0;
    }

    if (packet_size > len) {
        // Packet won't fit in buffer
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Packet too large %d > %d", packet_size, len);
        return 0;
    }

    // Step 2: Read the full packet in chunks (matching Adafruit EXACTLY)
    // CRITICAL: The header we just read is NOT part of the packet_size!
    // We need to read packet_size more bytes, but in the FIRST read, we get it directly.
    // In SUBSEQUENT reads, each I2C transaction returns another 4-byte header that we must skip.

    uint16_t cargo_remaining = packet_size;
    uint16_t cargo_read_amount = 0;
    bool first_read = true;
    uint8_t *pDest = pBuffer;

    // Temporary buffer for chunked reads
    uint8_t i2c_buffer[MAX_I2C_BUFFER];

    while (cargo_remaining > 0) {
        uint16_t read_size;
        if (first_read) {
            // First read after header: read as much cargo as possible (no extra header in this read)
            read_size = (cargo_remaining < MAX_I2C_BUFFER) ? cargo_remaining : MAX_I2C_BUFFER;
        } else {
            // Subsequent reads: each I2C read includes a 4-byte header that must be skipped
            read_size = ((cargo_remaining + 4) < MAX_I2C_BUFFER) ? (cargo_remaining + 4) : MAX_I2C_BUFFER;
        }

        if (!dev->transfer(nullptr, 0, i2c_buffer, read_size)) {
            return 0;
        }

        if (first_read) {
            // First read: entire buffer is cargo (no header to skip)
            cargo_read_amount = read_size;
            memcpy(pDest, i2c_buffer, cargo_read_amount);
            first_read = false;
        } else {
            // Subsequent reads: skip the 4-byte header at start of i2c_buffer
            cargo_read_amount = read_size - 4;
            memcpy(pDest, i2c_buffer + 4, cargo_read_amount);
        }

        pDest += cargo_read_amount;
        cargo_remaining -= cargo_read_amount;
    }

    return packet_size;
}

/**
 * hal_write - Write data to BNO08x
 *
 * Matches Adafruit i2chal_write() in Adafruit_BNO08x.cpp lines 389-404
 */
int AP_ExternalAHRS_BNO08x::hal_write(const uint8_t *pBuffer, unsigned len)
{
    if (!dev) {
        return 0;
    }

    // Write in chunks if needed (ESP32 I2C limit)
    uint16_t write_size = (len < MAX_I2C_BUFFER) ? len : MAX_I2C_BUFFER;

    if (!dev->transfer(pBuffer, write_size, nullptr, 0)) {
        return 0;
    }

    return write_size;
}

// ============================================================================
// SHTP Protocol Layer
// ============================================================================

/**
 * shtp_send_packet - Send SHTP packet
 *
 * Build and send a complete SHTP packet with header
 */
bool AP_ExternalAHRS_BNO08x::shtp_send_packet(uint8_t channel, const uint8_t *data, uint16_t length)
{
    if (length + SHTP_HDR_LEN > MAX_PACKET_SIZE) {
        return false;
    }

    uint16_t packet_len = length + SHTP_HDR_LEN;

    // Build SHTP header
    tx_buffer[0] = packet_len & 0xFF;           // Length LSB
    tx_buffer[1] = (packet_len >> 8) & 0x7F;    // Length MSB (clear continuation bit)
    tx_buffer[2] = channel;                      // Channel
    tx_buffer[3] = seq_num[channel]++;           // Sequence number (post-increment)

    // Copy payload
    if (length > 0 && data != nullptr) {
        memcpy(&tx_buffer[SHTP_HDR_LEN], data, length);
    }

    return hal_write(tx_buffer, packet_len) == packet_len;
}

/**
 * shtp_receive_packet - Receive SHTP packet
 *
 * Read a complete SHTP packet, parse header, store in rx_buffer
 * Sets rx_packet_len, rx_channel, rx_sequence
 *
 * @return true if packet received successfully
 */
bool AP_ExternalAHRS_BNO08x::shtp_receive_packet()
{
    int len = hal_read(rx_buffer, MAX_PACKET_SIZE);

    if (len < SHTP_HDR_LEN) {
        return false;
    }

    // Parse header
    rx_packet_len = (uint16_t)rx_buffer[0] | ((uint16_t)rx_buffer[1] << 8);
    rx_packet_len &= ~0x8000;  // Clear continuation bit
    rx_channel = rx_buffer[2];
    rx_sequence = rx_buffer[3];

    last_packet_ms = AP_HAL::millis();
    packet_count++;

    return true;
}

// ============================================================================
// Device Probe
// ============================================================================

bool AP_ExternalAHRS_BNO08x::probe_device(uint8_t bus, uint8_t addr)
{
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> probe_dev = hal.i2c_mgr->get_device_ptr(bus, addr);
    if (!probe_dev) {
        return false;
    }

    WITH_SEMAPHORE(probe_dev->get_semaphore());

    // Try to read 4 bytes (SHTP header)
    uint8_t dummy[4];
    return probe_dev->transfer(nullptr, 0, dummy, sizeof(dummy));
}

// ============================================================================
// Soft Reset - EXACT match with Adafruit i2chal_open()
// ============================================================================

/**
 * soft_reset_with_retry - Send soft reset with retries
 *
 * This EXACTLY matches Adafruit i2chal_open() in Adafruit_BNO08x.cpp lines 286-300
 *
 * Key points:
 * 1. Soft reset packet is EXACTLY: {5, 0, 1, 0, 1}
 * 2. Retry up to 5 times with 30ms delay between attempts
 * 3. Wait 300ms after successful reset
 * 4. Reset ALL sequence numbers to 0
 */
bool AP_ExternalAHRS_BNO08x::soft_reset_with_retry(uint8_t max_attempts)
{
    // EXACT soft reset packet from Adafruit i2chal_open()
    // Format: [length_lsb=5, length_msb=0, channel=1, sequence=0, command=1]
    // This is a RAW packet - NOT using shtp_send_packet() because sequence must be 0
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};

    // Reset all sequence numbers (like Adafruit does after reset)
    memset(seq_num, 0, sizeof(seq_num));

    reset_occurred = false;

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Sending soft reset...");

    // Try up to max_attempts times (Adafruit uses 5)
    bool success = false;
    for (uint8_t attempts = 0; attempts < max_attempts; attempts++) {
        if (hal_write(softreset_pkt, sizeof(softreset_pkt)) > 0) {
            success = true;
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Reset sent on attempt %d", attempts + 1);
            break;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Reset attempt %d failed, retrying...", attempts + 1);
        hal.scheduler->delay(30);  // 30ms delay between attempts (from Adafruit)
    }

    if (!success) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Soft reset failed after %d attempts", max_attempts);
        return false;
    }

    // CRITICAL: Wait 300ms after soft reset (from Adafruit i2chal_open)
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Waiting 300ms after reset...");
    hal.scheduler->delay(SOFTRESET_DELAY_MS);

    return true;
}

/**
 * wait_for_reset_complete - Wait for reset complete response
 *
 * After reset, BNO08x sends:
 * 1. Advertisement packets (channel 0)
 * 2. Reset complete response (channel 1, byte 4 = 0x01)
 *
 * We need to read and discard advertisements until we see reset complete
 */
bool AP_ExternalAHRS_BNO08x::wait_for_reset_complete()
{
    uint32_t start_ms = AP_HAL::millis();

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Waiting for reset complete...");

    while (AP_HAL::millis() - start_ms < RESET_TIMEOUT_MS) {
        hal.scheduler->delay(10);

        if (shtp_receive_packet()) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Got packet ch=%d len=%d", rx_channel, rx_packet_len);

            // Check for reset complete on executable channel (channel 1)
            if (rx_channel == SHTP_CHAN_EXECUTABLE && rx_packet_len >= 5) {
                uint8_t response = rx_buffer[SHTP_HDR_LEN];
                if (response == EXECUTABLE_RESET_COMPLETE) {
                    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Reset complete received!");
                    reset_count++;
                    return true;
                }
            }
            // Channel 0 = advertisements - continue reading
            // Channel 2 = control - continue reading
        }
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Timeout waiting for reset complete");
    return false;
}

// ============================================================================
// Product ID Request
// ============================================================================

bool AP_ExternalAHRS_BNO08x::get_product_ids()
{
    // Send Product ID Request (0xF9, 0x00)
    uint8_t prod_id_req[] = {SENSORHUB_PROD_ID_REQ, 0};

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Requesting product ID...");

    if (!shtp_send_packet(SHTP_CHAN_CONTROL, prod_id_req, sizeof(prod_id_req))) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Failed to send product ID request");
        return false;
    }

    // Wait for Product ID Response (0xF8)
    uint32_t start_ms = AP_HAL::millis();
    while (AP_HAL::millis() - start_ms < 1000) {
        hal.scheduler->delay(10);

        if (shtp_receive_packet()) {
            // Check for Product ID Response on control channel
            if (rx_channel == SHTP_CHAN_CONTROL &&
                rx_packet_len >= SHTP_HDR_LEN + 16 &&
                rx_buffer[SHTP_HDR_LEN] == SENSORHUB_PROD_ID_RESP) {

                const uint8_t *resp = &rx_buffer[SHTP_HDR_LEN];

                product_id.reset_cause = resp[1];
                product_id.sw_version_major = resp[2];
                product_id.sw_version_minor = resp[3];
                product_id.sw_part_number = read_u32_le(&resp[4]);
                product_id.sw_build_number = read_u32_le(&resp[8]);
                product_id.sw_version_patch = read_u16_le(&resp[12]);
                product_id.valid = true;

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: FW v%d.%d.%d (reset cause: %d)",
                              product_id.sw_version_major,
                              product_id.sw_version_minor,
                              product_id.sw_version_patch,
                              product_id.reset_cause);
                return true;
            }
        }
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Timeout waiting for product ID");
    return false;
}

// ============================================================================
// Sensor Configuration
// ============================================================================

/**
 * enable_report - Enable a sensor report
 *
 * Uses Set Feature Command (0xFD) as per SH-2 Reference Manual Section 6.5.4
 *
 * Format (17 bytes total):
 * [0]: Report ID (0xFD)
 * [1]: Feature Report ID (sensor ID to enable)
 * [2]: Feature flags
 * [3-4]: Change sensitivity (16-bit LE)
 * [5-8]: Report interval in microseconds (32-bit LE)
 * [9-12]: Batch interval in microseconds (32-bit LE)
 * [13-16]: Sensor-specific config (32-bit LE)
 */
bool AP_ExternalAHRS_BNO08x::enable_report(uint8_t sensor_id, uint32_t interval_us)
{
    uint8_t feature_cmd[17] = {0};

    feature_cmd[0] = SENSORHUB_SET_FEATURE_CMD;  // 0xFD
    feature_cmd[1] = sensor_id;
    feature_cmd[2] = 0;  // Flags: disabled (change sensitivity, wakeup, always-on)

    // Change sensitivity = 0 (bytes 3-4)
    feature_cmd[3] = 0;
    feature_cmd[4] = 0;

    // Report interval in microseconds (bytes 5-8, little endian)
    feature_cmd[5] = (interval_us >> 0) & 0xFF;
    feature_cmd[6] = (interval_us >> 8) & 0xFF;
    feature_cmd[7] = (interval_us >> 16) & 0xFF;
    feature_cmd[8] = (interval_us >> 24) & 0xFF;

    // Batch interval = 0 (bytes 9-12)
    // Sensor-specific = 0 (bytes 13-16)

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Enable sensor 0x%02X @ %luus", sensor_id, interval_us);

    return shtp_send_packet(SHTP_CHAN_CONTROL, feature_cmd, sizeof(feature_cmd));
}

bool AP_ExternalAHRS_BNO08x::enable_rotation_vector(uint32_t interval_us)
{
    return enable_report(SENSOR_ROTATION_VECTOR, interval_us);
}

bool AP_ExternalAHRS_BNO08x::enable_gyro(uint32_t interval_us)
{
    return enable_report(SENSOR_GYROSCOPE_CALIBRATED, interval_us);
}

bool AP_ExternalAHRS_BNO08x::enable_accelerometer(uint32_t interval_us)
{
    return enable_report(SENSOR_ACCELEROMETER, interval_us);
}

/**
 * configure_sensors - Enable all required sensor reports
 *
 * Matches the Arduino test code configuration
 */
bool AP_ExternalAHRS_BNO08x::configure_sensors()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: Configuring sensors...");

    // Enable Rotation Vector at 50Hz (20000us = 20ms interval)
    // Matching Arduino test code configuration (20000us = 50Hz update rate)
    if (!enable_rotation_vector(20000)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Failed to enable rotation vector");
        return false;
    }
    hal.scheduler->delay(20);

    // Enable Gyroscope at 50Hz
    if (!enable_gyro(20000)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Failed to enable gyroscope");
        return false;
    }
    hal.scheduler->delay(20);

    // Enable Accelerometer at 50Hz
    if (!enable_accelerometer(20000)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Failed to enable accelerometer");
        return false;
    }
    hal.scheduler->delay(20);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: Sensors configured (50Hz update rate)");
    return true;
}

// ============================================================================
// Reset Detection (like Adafruit wasReset)
// ============================================================================

/**
 * check_and_handle_reset - Check for unsolicited reset and reconfigure
 *
 * BNO08x can reset unexpectedly (power glitch, etc.)
 * When this happens, we need to reconfigure all sensors
 *
 * This matches Adafruit wasReset() + reconfiguration pattern
 */
bool AP_ExternalAHRS_BNO08x::check_and_handle_reset()
{
    if (!reset_occurred) {
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "BNO08x: Reset detected, reconfiguring...");

    reset_occurred = false;
    reset_count++;

    // Reconfigure all sensors
    if (!configure_sensors()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Failed to reconfigure after reset");
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: Reconfigured after reset");
    return true;
}

// ============================================================================
// Sensor Data Parsing
// ============================================================================

void AP_ExternalAHRS_BNO08x::process_packet()
{
    if (rx_packet_len < SHTP_HDR_LEN) {
        return;
    }

    // Debug: log packet info for first few packets
    static uint32_t debug_packet_count = 0;
    if (debug_packet_count < 20) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: RX ch=%d len=%d seq=%d",
                      rx_channel, rx_packet_len, rx_sequence);
        debug_packet_count++;
    }

    // Check for reset events on control channel
    if (rx_channel == SHTP_CHAN_CONTROL && rx_packet_len > SHTP_HDR_LEN) {
        uint8_t report_id = rx_buffer[SHTP_HDR_LEN];

        // Check for unsolicited initialize (reset indication)
        if (report_id == SENSORHUB_COMMAND_RESP) {
            parse_command_response(&rx_buffer[SHTP_HDR_LEN], rx_packet_len - SHTP_HDR_LEN);
        }
    }

    // Process sensor reports (channel 3)
    if (rx_channel == SHTP_CHAN_REPORTS && rx_packet_len > SHTP_HDR_LEN) {
        parse_sensor_reports(&rx_buffer[SHTP_HDR_LEN], rx_packet_len - SHTP_HDR_LEN);
    }
}

/**
 * parse_command_response - Check for unsolicited reset notification
 */
void AP_ExternalAHRS_BNO08x::parse_command_response(const uint8_t *data, uint16_t length)
{
    if (length < 5) {
        return;
    }

    uint8_t command = data[2];

    // Check for unsolicited initialize command (reset notification)
    // SH-2 sends this when it resets unexpectedly
    if (command == (SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED)) {
        uint8_t init_reason = data[4];  // r[1] in the response
        if (init_reason == SH2_INIT_SYSTEM) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "BNO08x: Unsolicited reset detected");
            reset_occurred = true;
        }
    }
}

/**
 * parse_sensor_reports - Parse multiple sensor reports from a packet
 */
void AP_ExternalAHRS_BNO08x::parse_sensor_reports(const uint8_t *data, uint16_t length)
{
    uint16_t cursor = 0;

    while (cursor < length) {
        uint8_t report_id = data[cursor];
        uint16_t report_len = 0;

        // Determine report length based on report ID
        switch (report_id) {
            case SENSOR_ACCELEROMETER:
            case SENSOR_GYROSCOPE_CALIBRATED:
            case SENSOR_MAGNETIC_FIELD:
            case SENSOR_LINEAR_ACCELERATION:
            case SENSOR_GRAVITY:
                report_len = 10;  // Standard 3-axis sensor report
                break;

            case SENSOR_ROTATION_VECTOR:
            case SENSOR_GEOMAG_ROTATION_VECTOR:
                report_len = 14;  // Quaternion + accuracy
                break;

            case SENSOR_GAME_ROTATION_VECTOR:
                report_len = 12;  // Quaternion without accuracy
                break;

            case SENSOR_GYROSCOPE_UNCALIBRATED:
                report_len = 16;  // 6-axis (calibrated + bias)
                break;

            case SENSOR_GYRO_INTEGRATED_RV:
                report_len = 14;  // Special format
                break;

            case SENSORHUB_BASE_TIMESTAMP:
                report_len = 5;  // Timestamp reference
                break;

            default:
                // Unknown report, skip remaining data
                return;
        }

        if (cursor + report_len > length) {
            break;
        }

        const uint8_t *report_data = &data[cursor];

        switch (report_id) {
            case SENSOR_ROTATION_VECTOR:
                parse_rotation_vector(report_data, report_len);
                break;

            case SENSOR_GAME_ROTATION_VECTOR:
                parse_game_rotation_vector(report_data, report_len);
                break;

            case SENSOR_GYROSCOPE_CALIBRATED:
                parse_gyro_calibrated(report_data, report_len);
                break;

            case SENSOR_ACCELEROMETER:
                parse_accelerometer(report_data, report_len);
                break;

            case SENSOR_LINEAR_ACCELERATION:
                parse_linear_acceleration(report_data, report_len);
                break;
        }

        cursor += report_len;
    }
}

/**
 * parse_rotation_vector - Parse Rotation Vector report (0x05)
 *
 * Report format (14 bytes):
 * [0]: Report ID (0x05)
 * [1]: Sequence number
 * [2]: Status (bits 1:0 = accuracy)
 * [3]: Delay LSB
 * [4-5]: Quaternion i (Q14)
 * [6-7]: Quaternion j (Q14)
 * [8-9]: Quaternion k (Q14)
 * [10-11]: Quaternion real (Q14)
 * [12-13]: Accuracy estimate (Q12, radians)
 *
 * COORDINATE SYSTEM CONVERSION:
 * BNO08x outputs ENU (East-North-Up) coordinate system
 * ArduPilot uses NED (North-East-Down) coordinate system
 *
 * ENU to NED quaternion conversion:
 * q_ned.w = q_enu.w
 * q_ned.x = q_enu.y  (NED X = ENU Y = North)
 * q_ned.y = q_enu.x  (NED Y = ENU X = East)
 * q_ned.z = -q_enu.z (NED Z = -ENU Z = Down)
 */
void AP_ExternalAHRS_BNO08x::parse_rotation_vector(const uint8_t *data, uint16_t length)
{
    if (length < 14) {
        return;
    }

    quat_status = data[2] & 0x03;

    // Extract quaternion components (Q14 format)
    // Data starts at byte 4 (after header: report_id, seq, status, delay)
    int16_t quat_i = read_i16_le(&data[4]);
    int16_t quat_j = read_i16_le(&data[6]);
    int16_t quat_k = read_i16_le(&data[8]);
    int16_t quat_real = read_i16_le(&data[10]);
    int16_t accuracy_raw = read_i16_le(&data[12]);

    // Convert from Q14 fixed point to float
    // BNO08x quaternion in ENU: (i, j, k, real) = (x_enu, y_enu, z_enu, w)
    float qx_enu = q_to_float(quat_i, ROTATION_VECTOR_Q);
    float qy_enu = q_to_float(quat_j, ROTATION_VECTOR_Q);
    float qz_enu = q_to_float(quat_k, ROTATION_VECTOR_Q);
    float qw = q_to_float(quat_real, ROTATION_VECTOR_Q);

    // Convert ENU to NED coordinate system for ArduPilot
    // NED quaternion: swap x<->y and negate z
    float qx_ned = qy_enu;   // NED X (North) = ENU Y
    float qy_ned = qx_enu;   // NED Y (East) = ENU X
    float qz_ned = -qz_enu;  // NED Z (Down) = -ENU Z (Up)

    // ArduPilot Quaternion: (q1, q2, q3, q4) = (w, x, y, z)
    {
        WITH_SEMAPHORE(data_sem);
        latest_quat[0] = qw;      // w (unchanged)
        latest_quat[1] = qx_ned;  // x (NED)
        latest_quat[2] = qy_ned;  // y (NED)
        latest_quat[3] = qz_ned;  // z (NED)
        latest_quat.normalize();

        quat_accuracy_rad = q_to_float(accuracy_raw, ROTATION_ACCURACY_Q);
        have_rotation = true;
    }

    last_rotation_ms = AP_HAL::millis();

    // Debug: log quaternion and euler angles for verification
    static uint8_t log_count = 0;
    if (log_count < 15) {
        // Calculate euler angles from NED quaternion
        float roll, pitch, yaw;
        latest_quat.to_euler(roll, pitch, yaw);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: R=%.1f P=%.1f Y=%.1f deg",
                      (double)degrees(roll), (double)degrees(pitch), (double)degrees(yaw));
        log_count++;
    }
}

void AP_ExternalAHRS_BNO08x::parse_game_rotation_vector(const uint8_t *data, uint16_t length)
{
    if (length < 12) {
        return;
    }

    quat_status = data[2] & 0x03;

    int16_t quat_i = read_i16_le(&data[4]);
    int16_t quat_j = read_i16_le(&data[6]);
    int16_t quat_k = read_i16_le(&data[8]);
    int16_t quat_real = read_i16_le(&data[10]);

    // BNO08x quaternion in ENU
    float qx_enu = q_to_float(quat_i, ROTATION_VECTOR_Q);
    float qy_enu = q_to_float(quat_j, ROTATION_VECTOR_Q);
    float qz_enu = q_to_float(quat_k, ROTATION_VECTOR_Q);
    float qw = q_to_float(quat_real, ROTATION_VECTOR_Q);

    // Convert ENU to NED
    float qx_ned = qy_enu;
    float qy_ned = qx_enu;
    float qz_ned = -qz_enu;

    {
        WITH_SEMAPHORE(data_sem);
        latest_quat[0] = qw;
        latest_quat[1] = qx_ned;
        latest_quat[2] = qy_ned;
        latest_quat[3] = qz_ned;
        latest_quat.normalize();

        quat_accuracy_rad = 0;  // No accuracy for game rotation vector
        have_rotation = true;
    }

    last_rotation_ms = AP_HAL::millis();
}

void AP_ExternalAHRS_BNO08x::parse_gyro_calibrated(const uint8_t *data, uint16_t length)
{
    if (length < 10) {
        return;
    }

    int16_t gyro_x_raw = read_i16_le(&data[4]);
    int16_t gyro_y_raw = read_i16_le(&data[6]);
    int16_t gyro_z_raw = read_i16_le(&data[8]);

    // BNO08x gyro in ENU (rad/s)
    float gx_enu = q_to_float(gyro_x_raw, GYROSCOPE_Q);
    float gy_enu = q_to_float(gyro_y_raw, GYROSCOPE_Q);
    float gz_enu = q_to_float(gyro_z_raw, GYROSCOPE_Q);

    // Convert ENU to NED
    // NED.x = ENU.y, NED.y = ENU.x, NED.z = -ENU.z
    {
        WITH_SEMAPHORE(data_sem);
        latest_gyro.x = gy_enu;   // NED X = ENU Y
        latest_gyro.y = gx_enu;   // NED Y = ENU X
        latest_gyro.z = -gz_enu;  // NED Z = -ENU Z
        have_gyro_data = true;
    }

    last_gyro_ms = AP_HAL::millis();
}

void AP_ExternalAHRS_BNO08x::parse_accelerometer(const uint8_t *data, uint16_t length)
{
    if (length < 10) {
        return;
    }

    int16_t accel_x_raw = read_i16_le(&data[4]);
    int16_t accel_y_raw = read_i16_le(&data[6]);
    int16_t accel_z_raw = read_i16_le(&data[8]);

    // BNO08x accel in ENU (m/s^2)
    float ax_enu = q_to_float(accel_x_raw, ACCELEROMETER_Q);
    float ay_enu = q_to_float(accel_y_raw, ACCELEROMETER_Q);
    float az_enu = q_to_float(accel_z_raw, ACCELEROMETER_Q);

    // Convert ENU to NED
    // NED.x = ENU.y, NED.y = ENU.x, NED.z = -ENU.z
    {
        WITH_SEMAPHORE(data_sem);
        latest_accel.x = ay_enu;   // NED X = ENU Y
        latest_accel.y = ax_enu;   // NED Y = ENU X
        latest_accel.z = -az_enu;  // NED Z = -ENU Z
        have_accel_data = true;
    }

    last_accel_ms = AP_HAL::millis();
}

void AP_ExternalAHRS_BNO08x::parse_linear_acceleration(const uint8_t *data, uint16_t length)
{
    if (length < 10) {
        return;
    }

    int16_t accel_x_raw = read_i16_le(&data[4]);
    int16_t accel_y_raw = read_i16_le(&data[6]);
    int16_t accel_z_raw = read_i16_le(&data[8]);

    // BNO08x linear accel in ENU (m/s^2)
    float ax_enu = q_to_float(accel_x_raw, LINEAR_ACCEL_Q);
    float ay_enu = q_to_float(accel_y_raw, LINEAR_ACCEL_Q);
    float az_enu = q_to_float(accel_z_raw, LINEAR_ACCEL_Q);

    // Convert ENU to NED
    {
        WITH_SEMAPHORE(data_sem);
        latest_linear_accel.x = ay_enu;   // NED X = ENU Y
        latest_linear_accel.y = ax_enu;   // NED Y = ENU X
        latest_linear_accel.z = -az_enu;  // NED Z = -ENU Z
    }
}

// ============================================================================
// Main Initialization (runs in thread context)
// ============================================================================

/**
 * init_sensor - Initialize BNO08x sensor
 *
 * This follows the EXACT sequence from the WORKING Arduino code:
 * 1. Wire.begin(SDA_PIN, SCL_PIN)
 * 2. Wire.setClock(400000)
 * 3. delay(500)  <-- CRITICAL: Wait for BNO08x to boot
 * 4. bno08x.begin_I2C(BNO_ADDR, &Wire) with retries
 * 5. bno08x.enableReport(...)
 *
 * Returns true if initialization successful
 */
bool AP_ExternalAHRS_BNO08x::init_sensor()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: Starting initialization...");

    // Initialize INT pin
    if (!init_int_pin()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "BNO08x: INT pin init failed, using polling");
    }

    // CRITICAL: Wait 500ms for BNO08x to boot (from Arduino test code)
    // This is often missing in implementations and causes initialization failures
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Waiting %lums for sensor boot...", STARTUP_DELAY_MS);
    hal.scheduler->delay(STARTUP_DELAY_MS);

    // Probe for device at both addresses
    uint8_t found_addr = 0;
    if (probe_device(0, BNO08X_I2C_ADDR_ALT)) {
        found_addr = BNO08X_I2C_ADDR_ALT;  // Try 0x4B first (user's hardware)
    } else if (probe_device(0, BNO08X_I2C_ADDR)) {
        found_addr = BNO08X_I2C_ADDR;  // Try 0x4A
    }

    if (found_addr == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: No device found at 0x4A or 0x4B");
        return false;
    }

    // Get I2C device handle
    dev = hal.i2c_mgr->get_device_ptr(0, found_addr);
    if (!dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Failed to get I2C device");
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: Found at 0x%02X", found_addr);

    // Try initialization up to MAX_INIT_ATTEMPTS times (matching Arduino retry logic)
    for (uint8_t attempt = 0; attempt < MAX_INIT_ATTEMPTS; attempt++) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Init attempt %d/%d", attempt + 1, MAX_INIT_ATTEMPTS);

        WITH_SEMAPHORE(dev->get_semaphore());

        // Step 1: Soft reset with retries (matching Adafruit i2chal_open)
        if (!soft_reset_with_retry(5)) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Soft reset failed, retrying init...");
            hal.scheduler->delay(100);
            continue;
        }

        // Step 2: Wait for reset complete
        if (!wait_for_reset_complete()) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Reset complete timeout, retrying init...");
            hal.scheduler->delay(100);
            continue;
        }

        // Step 3: Get product IDs (verifies communication)
        if (!get_product_ids()) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Product ID failed, retrying init...");
            hal.scheduler->delay(100);
            continue;
        }

        // Step 4: Configure sensor reports
        if (!configure_sensors()) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Sensor config failed, retrying init...");
            hal.scheduler->delay(100);
            continue;
        }

        // Success!
        sensor_initialized = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: Initialized successfully on attempt %d", attempt + 1);
        return true;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Initialization failed after %d attempts", MAX_INIT_ATTEMPTS);
    return false;
}

// ============================================================================
// Update Thread (runs I2C operations)
// ============================================================================

void AP_ExternalAHRS_BNO08x::update_thread()
{
    // Wait for system to be ready
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(10);
    }

    thread_started = true;
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "BNO08x: Thread started");

    // Initialize sensor (blocking is OK in thread context)
    if (!init_sensor()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Thread init failed, thread exiting");
        return;
    }

    // Main sensor read loop
    // CRITICAL: For I2C mode, we do NOT use INT pin! (INT is only for SPI mode in Adafruit lib)
    // Instead, we continuously poll by attempting to read, just like Adafruit getSensorEvent() does.
    // The BNO08x will return valid data when available, or an empty/invalid packet when not ready.
    while (true) {
        if (!dev) {
            hal.scheduler->delay(100);
            continue;
        }

        // Check for and handle unexpected resets (like Adafruit wasReset)
        if (reset_occurred) {
            WITH_SEMAPHORE(dev->get_semaphore());
            check_and_handle_reset();
        }

        // Read packet with I2C semaphore
        // This is like calling sh2_service() -> shtp_service() -> pHal->read()
        bool got_packet = false;
        {
            WITH_SEMAPHORE(dev->get_semaphore());
            got_packet = shtp_receive_packet();
        }

        // Process packet (parsing doesn't need I2C semaphore)
        if (got_packet) {
            process_packet();
        }

        // Small delay to match Arduino test code (delay(5) in loop)
        // This prevents excessive I2C traffic while still being responsive
        hal.scheduler->delay(5);
    }
}

// ============================================================================
// Main Update (called from scheduler - NO I2C operations!)
// ============================================================================

void AP_ExternalAHRS_BNO08x::update()
{
    if (!sensor_initialized) {
        return;
    }

    // Copy data from thread to main state with proper locking
    bool copy_rotation = false;
    bool copy_gyro = false;
    bool copy_accel = false;
    Quaternion quat_copy;
    Vector3f gyro_copy;
    Vector3f accel_copy;

    {
        WITH_SEMAPHORE(data_sem);
        if (have_rotation) {
            quat_copy = latest_quat;
            copy_rotation = true;
            have_rotation = false;
        }
        if (have_gyro_data) {
            gyro_copy = latest_gyro;
            copy_gyro = true;
            have_gyro_data = false;
        }
        if (have_accel_data) {
            accel_copy = latest_accel;
            copy_accel = true;
            have_accel_data = false;
        }
    }

    // Update external AHRS state
    if (copy_rotation || copy_gyro || copy_accel) {
        WITH_SEMAPHORE(state.sem);

        if (copy_rotation) {
            state.quat = quat_copy;
            state.have_quaternion = true;
        }
        if (copy_gyro) {
            state.gyro = gyro_copy;
        }
        if (copy_accel) {
            state.accel = accel_copy;
        }
    }
}

// ============================================================================
// Status Functions
// ============================================================================

bool AP_ExternalAHRS_BNO08x::healthy(void) const
{
    if (!sensor_initialized) {
        return false;
    }

    uint32_t now = AP_HAL::millis();
    return (now - last_packet_ms < SENSOR_TIMEOUT_MS);
}

bool AP_ExternalAHRS_BNO08x::initialised(void) const
{
    return sensor_initialized;
}

bool AP_ExternalAHRS_BNO08x::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!sensor_initialized) {
        hal.util->snprintf(failure_msg, failure_msg_len, "BNO08x not initialized");
        return false;
    }

    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "BNO08x no data (last: %lums ago)",
                           AP_HAL::millis() - last_packet_ms);
        return false;
    }

    if (!product_id.valid) {
        hal.util->snprintf(failure_msg, failure_msg_len, "BNO08x ID invalid");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_BNO08x::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));

    if (sensor_initialized && healthy()) {
        status.flags.attitude = 1;
        status.flags.horiz_vel = 0;
        status.flags.vert_vel = 0;
        status.flags.horiz_pos_rel = 0;
        status.flags.horiz_pos_abs = 0;
        status.flags.vert_pos = 0;
    }
}

#endif  // AP_EXTERNAL_AHRS_BNO08X_ENABLED
