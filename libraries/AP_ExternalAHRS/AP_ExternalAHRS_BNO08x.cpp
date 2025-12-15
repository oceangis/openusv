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
  ESP32-S3 specific implementation - IMPROVED VERSION

  Based on analysis of:
  - Adafruit BNO08x Library (Hillcrest SH2 Driver)
  - SparkFun BNO08x Library
  - BNO08x Datasheet and SH-2 Reference Manual
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_BNO08X_ENABLED

#include "AP_ExternalAHRS_BNO08x.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// ============================================================================
// Constructor
// ============================================================================

AP_ExternalAHRS_BNO08x::AP_ExternalAHRS_BNO08x(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state)
    : AP_ExternalAHRS_backend(frontend, state)
    , sensor_initialized(false)
    , reset_complete(false)
    , last_packet_ms(0)
    , last_rotation_ms(0)
    , last_gyro_ms(0)
    , last_accel_ms(0)
    , sensor_timestamp_us(0)
    , quat_accuracy_rad(0)
    , quat_status(0)
    , have_rotation(false)
    , have_gyro_data(false)
    , have_accel_data(false)
{
    // Initialize sequence numbers
    memset(seq_num, 0, sizeof(seq_num));

    // Initialize product ID
    memset(&product_id, 0, sizeof(product_id));
}

// ============================================================================
// Utility Functions (matching Hillcrest sh2_util)
// ============================================================================

/**
 * Convert Q-format fixed point to float
 * From Hillcrest SH2 driver
 */
float AP_ExternalAHRS_BNO08x::q_to_float(int16_t fixed_point, uint8_t q_point)
{
    return (float)fixed_point / (float)(1 << q_point);
}

/**
 * Read 16-bit signed little-endian value
 */
int16_t AP_ExternalAHRS_BNO08x::read_i16_le(const uint8_t *buffer)
{
    return (int16_t)(buffer[0] | (buffer[1] << 8));
}

/**
 * Read 16-bit unsigned little-endian value
 */
uint16_t AP_ExternalAHRS_BNO08x::read_u16_le(const uint8_t *buffer)
{
    return (uint16_t)(buffer[0] | (buffer[1] << 8));
}

/**
 * Read 32-bit unsigned little-endian value
 */
uint32_t AP_ExternalAHRS_BNO08x::read_u32_le(const uint8_t *buffer)
{
    return (uint32_t)(buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24));
}

// ============================================================================
// I2C Low-Level Functions
// ============================================================================

/**
 * Read SHTP packet from I2C
 * Following Adafruit/SparkFun pattern for handling multi-part reads
 */
bool AP_ExternalAHRS_BNO08x::i2c_read_packet(uint8_t *buffer, uint16_t max_len, uint16_t &actual_len)
{
    if (!dev) {
        return false;
    }

    // Step 1: Read 4-byte header to get packet length
    uint8_t header[SHTP_HDR_LEN];
    if (!dev->transfer(nullptr, 0, header, SHTP_HDR_LEN)) {
        return false;
    }

    // Extract packet length (mask off continuation bit)
    uint16_t packet_len = read_u16_le(header) & 0x7FFF;

    // Sanity checks
    if (packet_len == 0 || packet_len == 0x7FFF) {
        // No data or invalid
        return false;
    }

    if (packet_len > max_len) {
        // Packet too large for buffer
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "BNO08x: Packet too large (%u > %u)", packet_len, max_len);
        return false;
    }

    // Step 2: Read the full packet including header
    // BNO08x I2C: Each read returns header + cargo
    // For subsequent reads, header is repeated
    uint16_t cargo_remaining = packet_len;
    uint16_t buffer_pos = 0;
    bool first_read = true;

    while (cargo_remaining > 0) {
        uint16_t read_size;
        if (first_read) {
            // First read: get up to max transfer bytes
            read_size = (cargo_remaining < MAX_I2C_TRANSFER) ? cargo_remaining : MAX_I2C_TRANSFER;
        } else {
            // Subsequent reads include a 4-byte header
            read_size = (cargo_remaining + SHTP_HDR_LEN < MAX_I2C_TRANSFER) ?
                        (cargo_remaining + SHTP_HDR_LEN) : MAX_I2C_TRANSFER;
        }

        uint8_t temp_buf[MAX_I2C_TRANSFER];
        if (!dev->transfer(nullptr, 0, temp_buf, read_size)) {
            return false;
        }

        if (first_read) {
            // First read: copy everything (header + data)
            memcpy(buffer + buffer_pos, temp_buf, read_size);
            buffer_pos += read_size;
            cargo_remaining -= read_size;
            first_read = false;
        } else {
            // Subsequent reads: skip the repeated header (4 bytes)
            uint16_t cargo_in_this_read = read_size - SHTP_HDR_LEN;
            memcpy(buffer + buffer_pos, temp_buf + SHTP_HDR_LEN, cargo_in_this_read);
            buffer_pos += cargo_in_this_read;
            cargo_remaining -= cargo_in_this_read;
        }
    }

    actual_len = packet_len;
    return true;
}

/**
 * Write SHTP packet to I2C
 */
bool AP_ExternalAHRS_BNO08x::i2c_write_packet(const uint8_t *buffer, uint16_t len)
{
    if (!dev || len > MAX_I2C_TRANSFER) {
        return false;
    }

    return dev->transfer(buffer, len, nullptr, 0);
}

// ============================================================================
// SHTP Protocol Layer
// ============================================================================

/**
 * Send SHTP packet
 * Channel: SHTP channel (0-5)
 * Data: Payload data (without SHTP header)
 * Length: Payload length
 */
bool AP_ExternalAHRS_BNO08x::shtp_send(uint8_t channel, const uint8_t *data, uint16_t length)
{
    if (length + SHTP_HDR_LEN > MAX_PACKET_SIZE) {
        return false;
    }

    // Build SHTP packet: [len_lsb, len_msb, channel, sequence, ...payload...]
    uint16_t packet_len = length + SHTP_HDR_LEN;

    tx_buffer[0] = packet_len & 0xFF;
    tx_buffer[1] = (packet_len >> 8) & 0x7F;  // Clear continuation bit
    tx_buffer[2] = channel;
    tx_buffer[3] = seq_num[channel]++;  // Increment sequence number

    if (length > 0 && data != nullptr) {
        memcpy(&tx_buffer[SHTP_HDR_LEN], data, length);
    }

    return i2c_write_packet(tx_buffer, packet_len);
}

/**
 * Receive SHTP packet
 * Returns true if packet received, fills packet buffer and length
 */
bool AP_ExternalAHRS_BNO08x::shtp_receive(uint8_t *packet, uint16_t &length, uint32_t *timestamp_us)
{
    if (!i2c_read_packet(packet, MAX_PACKET_SIZE, length)) {
        return false;
    }

    if (length < SHTP_HDR_LEN) {
        return false;
    }

    // Update sequence tracking
    uint8_t channel = packet[2];
    // uint8_t seq = packet[3];
    // Could track expected sequence here

    if (timestamp_us) {
        *timestamp_us = AP_HAL::micros();
    }

    last_packet_ms = AP_HAL::millis();
    return true;
}

// ============================================================================
// Initialization
// ============================================================================

/**
 * Initialize the BNO08x sensor
 */
bool AP_ExternalAHRS_BNO08x::init()
{
    // Try primary I2C address first
    dev = hal.i2c_mgr->get_device_ptr(0, BNO08X_I2C_ADDR);

    if (!dev) {
        // Try alternate address
        dev = hal.i2c_mgr->get_device_ptr(0, BNO08X_I2C_ADDR_ALT);
        if (!dev) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: I2C device allocation failed");
            return false;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: Using alternate address 0x4B");
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    // Wait for sensor power-up
    hal.scheduler->delay(100);

    // Perform soft reset
    if (!soft_reset()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Soft reset failed");
        return false;
    }

    // Wait for reset complete
    if (!wait_for_reset_complete()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Reset did not complete");
        return false;
    }

    // Get product IDs to verify communication
    if (!get_product_ids()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Failed to get product IDs");
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: FW v%d.%d.%d",
                  product_id.sw_version_major,
                  product_id.sw_version_minor,
                  product_id.sw_version_patch);

    // Configure sensor reports
    if (!configure_sensor()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Configuration failed");
        return false;
    }

    sensor_initialized = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: Initialized successfully");

    return true;
}

/**
 * Send soft reset command
 */
bool AP_ExternalAHRS_BNO08x::soft_reset()
{
    // Soft reset packet for executable channel
    // Format: [Length=5, 0, Channel=1, Seq=0, Cmd=1]
    // The Adafruit library sends this as a raw 5-byte packet
    uint8_t reset_cmd[] = {EXECUTABLE_CMD_RESET};

    reset_complete = false;

    // Send on executable channel
    return shtp_send(SHTP_CHAN_EXECUTABLE, reset_cmd, sizeof(reset_cmd));
}

/**
 * Wait for reset complete response
 */
bool AP_ExternalAHRS_BNO08x::wait_for_reset_complete()
{
    uint32_t start_ms = AP_HAL::millis();

    // Wait for reset complete, with timeout
    while (AP_HAL::millis() - start_ms < RESET_TIMEOUT_MS) {
        hal.scheduler->delay(10);

        uint16_t length;
        if (shtp_receive(rx_buffer, length, nullptr)) {
            // Check for reset complete on executable channel
            if (length >= 5 && rx_buffer[2] == SHTP_CHAN_EXECUTABLE) {
                uint8_t response = rx_buffer[SHTP_HDR_LEN];
                if (response == EXECUTABLE_RESET_COMPLETE) {
                    reset_complete = true;
                    return true;
                }
            }
        }
    }

    return false;
}

/**
 * Request and parse product IDs
 */
bool AP_ExternalAHRS_BNO08x::get_product_ids()
{
    // Send Product ID Request
    uint8_t prod_id_req[] = {SENSORHUB_PROD_ID_REQ, 0};  // Report ID + Reserved

    if (!shtp_send(SHTP_CHAN_CONTROL, prod_id_req, sizeof(prod_id_req))) {
        return false;
    }

    // Wait for response
    uint32_t start_ms = AP_HAL::millis();
    while (AP_HAL::millis() - start_ms < 500) {
        hal.scheduler->delay(10);

        uint16_t length;
        if (shtp_receive(rx_buffer, length, nullptr)) {
            // Check for Product ID Response on control channel
            if (length >= SHTP_HDR_LEN + 16 &&
                rx_buffer[2] == SHTP_CHAN_CONTROL &&
                rx_buffer[SHTP_HDR_LEN] == SENSORHUB_PROD_ID_RESP) {

                const uint8_t *resp = &rx_buffer[SHTP_HDR_LEN];

                product_id.reset_cause = resp[1];
                product_id.sw_version_major = resp[2];
                product_id.sw_version_minor = resp[3];
                product_id.sw_part_number = read_u32_le(&resp[4]);
                product_id.sw_build_number = read_u32_le(&resp[8]);
                product_id.sw_version_patch = read_u16_le(&resp[12]);
                product_id.valid = true;

                return true;
            }
        }
    }

    return false;
}

/**
 * Configure sensor to enable required reports
 */
bool AP_ExternalAHRS_BNO08x::configure_sensor()
{
    // Enable Rotation Vector at 100Hz
    if (!enable_rotation_vector(DEFAULT_REPORT_INTERVAL_US)) {
        return false;
    }
    hal.scheduler->delay(20);

    // Enable Calibrated Gyroscope at 100Hz
    if (!enable_gyro(DEFAULT_REPORT_INTERVAL_US)) {
        return false;
    }
    hal.scheduler->delay(20);

    // Enable Accelerometer at 100Hz
    if (!enable_accelerometer(DEFAULT_REPORT_INTERVAL_US)) {
        return false;
    }
    hal.scheduler->delay(20);

    return true;
}

// ============================================================================
// Sensor Configuration
// ============================================================================

/**
 * Enable a sensor report at specified interval
 * Uses Set Feature Command (0xFD)
 *
 * Set Feature Command format (17 bytes):
 * [0]: Report ID (0xFD)
 * [1]: Feature Report ID (sensor ID)
 * [2]: Feature flags
 * [3-4]: Change sensitivity (16-bit LE)
 * [5-8]: Report interval in microseconds (32-bit LE)
 * [9-12]: Batch interval in microseconds (32-bit LE)
 * [13-16]: Sensor-specific config (32-bit LE)
 */
bool AP_ExternalAHRS_BNO08x::enable_report(uint8_t sensor_id, uint32_t report_interval_us)
{
    uint8_t feature_cmd[17] = {0};

    feature_cmd[0] = SENSORHUB_SET_FEATURE_CMD;  // 0xFD
    feature_cmd[1] = sensor_id;
    feature_cmd[2] = 0;  // Flags: no wakeup, no sensitivity change

    // Change sensitivity (bytes 3-4)
    feature_cmd[3] = 0;
    feature_cmd[4] = 0;

    // Report interval in microseconds (bytes 5-8, little endian)
    feature_cmd[5] = (report_interval_us >> 0) & 0xFF;
    feature_cmd[6] = (report_interval_us >> 8) & 0xFF;
    feature_cmd[7] = (report_interval_us >> 16) & 0xFF;
    feature_cmd[8] = (report_interval_us >> 24) & 0xFF;

    // Batch interval (bytes 9-12) - set to 0
    feature_cmd[9] = 0;
    feature_cmd[10] = 0;
    feature_cmd[11] = 0;
    feature_cmd[12] = 0;

    // Sensor-specific (bytes 13-16) - set to 0
    feature_cmd[13] = 0;
    feature_cmd[14] = 0;
    feature_cmd[15] = 0;
    feature_cmd[16] = 0;

    // Send on Sensorhub Control channel (channel 2)
    return shtp_send(SHTP_CHAN_CONTROL, feature_cmd, sizeof(feature_cmd));
}

bool AP_ExternalAHRS_BNO08x::enable_rotation_vector(uint32_t report_interval_us)
{
    return enable_report(SENSOR_ROTATION_VECTOR, report_interval_us);
}

bool AP_ExternalAHRS_BNO08x::enable_gyro(uint32_t report_interval_us)
{
    return enable_report(SENSOR_GYROSCOPE_CALIBRATED, report_interval_us);
}

bool AP_ExternalAHRS_BNO08x::enable_accelerometer(uint32_t report_interval_us)
{
    return enable_report(SENSOR_ACCELEROMETER, report_interval_us);
}

bool AP_ExternalAHRS_BNO08x::enable_linear_acceleration(uint32_t report_interval_us)
{
    return enable_report(SENSOR_LINEAR_ACCELERATION, report_interval_us);
}

// ============================================================================
// Sensor Data Parsing
// ============================================================================

/**
 * Process received SHTP packet
 */
void AP_ExternalAHRS_BNO08x::process_packet(uint8_t *packet, uint16_t length)
{
    if (length < SHTP_HDR_LEN) {
        return;
    }

    uint8_t channel = packet[2];

    // We're mainly interested in sensor reports (channel 3)
    if (channel == SHTP_CHAN_REPORTS) {
        parse_sensor_reports(&packet[SHTP_HDR_LEN], length - SHTP_HDR_LEN);
    }
}

/**
 * Parse sensor input reports
 * Multiple reports can be concatenated in a single packet
 */
void AP_ExternalAHRS_BNO08x::parse_sensor_reports(uint8_t *data, uint16_t length)
{
    uint16_t cursor = 0;

    while (cursor < length) {
        uint8_t report_id = data[cursor];

        // Determine report length based on report ID
        // These are the minimum lengths for each report type
        uint16_t report_len = 0;

        switch (report_id) {
            case SENSOR_ACCELEROMETER:
            case SENSOR_GYROSCOPE_CALIBRATED:
            case SENSOR_MAGNETIC_FIELD:
            case SENSOR_LINEAR_ACCELERATION:
            case SENSOR_GRAVITY:
                report_len = 10;  // Report ID + Seq + Status + Delay + 3x16bit
                break;

            case SENSOR_ROTATION_VECTOR:
            case SENSOR_GEOMAG_ROTATION_VECTOR:
                report_len = 14;  // Report ID + Seq + Status + Delay + 4x16bit + accuracy
                break;

            case SENSOR_GAME_ROTATION_VECTOR:
                report_len = 12;  // Report ID + Seq + Status + Delay + 4x16bit (no accuracy)
                break;

            case SENSOR_GYROSCOPE_UNCALIBRATED:
                report_len = 16;  // Report ID + Seq + Status + Delay + 6x16bit
                break;

            case SENSOR_GYRO_INTEGRATED_RV:
                report_len = 14;  // Special format: no Seq/Status bytes
                break;

            case SENSORHUB_BASE_TIMESTAMP:
                // Base timestamp reference (5 bytes)
                report_len = 5;
                break;

            default:
                // Unknown report, skip remaining data
                return;
        }

        // Safety check
        if (cursor + report_len > length) {
            break;
        }

        // Parse the specific report
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

            default:
                break;
        }

        cursor += report_len;
    }
}

/**
 * Parse Rotation Vector report (0x05)
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
 */
void AP_ExternalAHRS_BNO08x::parse_rotation_vector(const uint8_t *data, uint16_t length)
{
    if (length < 14) {
        return;
    }

    // Extract status
    quat_status = data[2] & 0x03;

    // Extract quaternion components (Q14 format)
    // NOTE: Corrected offsets - data starts at byte 4, not byte 5
    int16_t quat_i = read_i16_le(&data[4]);
    int16_t quat_j = read_i16_le(&data[6]);
    int16_t quat_k = read_i16_le(&data[8]);
    int16_t quat_real = read_i16_le(&data[10]);
    int16_t accuracy_raw = read_i16_le(&data[12]);

    // Convert to float
    float qi = q_to_float(quat_i, ROTATION_VECTOR_Q);
    float qj = q_to_float(quat_j, ROTATION_VECTOR_Q);
    float qk = q_to_float(quat_k, ROTATION_VECTOR_Q);
    float qw = q_to_float(quat_real, ROTATION_VECTOR_Q);

    // BNO08x quaternion: (i, j, k, real) = (x, y, z, w)
    // ArduPilot Quaternion: (q1, q2, q3, q4) = (w, x, y, z)
    latest_quat[0] = qw;  // w
    latest_quat[1] = qi;  // x (i)
    latest_quat[2] = qj;  // y (j)
    latest_quat[3] = qk;  // z (k)

    // Normalize quaternion
    latest_quat.normalize();

    // Accuracy in radians (Q12)
    quat_accuracy_rad = q_to_float(accuracy_raw, ROTATION_ACCURACY_Q);

    have_rotation = true;
    last_rotation_ms = AP_HAL::millis();
}

/**
 * Parse Game Rotation Vector report (0x08)
 * Same as rotation vector but without accuracy estimate
 */
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

    float qi = q_to_float(quat_i, ROTATION_VECTOR_Q);
    float qj = q_to_float(quat_j, ROTATION_VECTOR_Q);
    float qk = q_to_float(quat_k, ROTATION_VECTOR_Q);
    float qw = q_to_float(quat_real, ROTATION_VECTOR_Q);

    latest_quat[0] = qw;
    latest_quat[1] = qi;
    latest_quat[2] = qj;
    latest_quat[3] = qk;
    latest_quat.normalize();

    quat_accuracy_rad = 0;  // No accuracy for game rotation vector

    have_rotation = true;
    last_rotation_ms = AP_HAL::millis();
}

/**
 * Parse Calibrated Gyroscope report (0x02)
 *
 * Report format (10 bytes):
 * [0]: Report ID (0x02)
 * [1]: Sequence number
 * [2]: Status
 * [3]: Delay LSB
 * [4-5]: X axis (Q9, rad/s)
 * [6-7]: Y axis (Q9, rad/s)
 * [8-9]: Z axis (Q9, rad/s)
 */
void AP_ExternalAHRS_BNO08x::parse_gyro_calibrated(const uint8_t *data, uint16_t length)
{
    if (length < 10) {
        return;
    }

    // NOTE: Corrected offsets - data starts at byte 4
    int16_t gyro_x = read_i16_le(&data[4]);
    int16_t gyro_y = read_i16_le(&data[6]);
    int16_t gyro_z = read_i16_le(&data[8]);

    // Convert from Q9 to rad/s
    latest_gyro.x = q_to_float(gyro_x, GYROSCOPE_Q);
    latest_gyro.y = q_to_float(gyro_y, GYROSCOPE_Q);
    latest_gyro.z = q_to_float(gyro_z, GYROSCOPE_Q);

    have_gyro_data = true;
    last_gyro_ms = AP_HAL::millis();
}

/**
 * Parse Accelerometer report (0x01)
 *
 * Report format (10 bytes):
 * [0]: Report ID (0x01)
 * [1]: Sequence number
 * [2]: Status
 * [3]: Delay LSB
 * [4-5]: X axis (Q8, m/s^2)
 * [6-7]: Y axis (Q8, m/s^2)
 * [8-9]: Z axis (Q8, m/s^2)
 */
void AP_ExternalAHRS_BNO08x::parse_accelerometer(const uint8_t *data, uint16_t length)
{
    if (length < 10) {
        return;
    }

    // NOTE: Corrected offsets - data starts at byte 4
    int16_t accel_x = read_i16_le(&data[4]);
    int16_t accel_y = read_i16_le(&data[6]);
    int16_t accel_z = read_i16_le(&data[8]);

    // Convert from Q8 to m/s^2
    latest_accel.x = q_to_float(accel_x, ACCELEROMETER_Q);
    latest_accel.y = q_to_float(accel_y, ACCELEROMETER_Q);
    latest_accel.z = q_to_float(accel_z, ACCELEROMETER_Q);

    have_accel_data = true;
    last_accel_ms = AP_HAL::millis();
}

/**
 * Parse Linear Acceleration report (0x04)
 * Same format as accelerometer but gravity-compensated
 */
void AP_ExternalAHRS_BNO08x::parse_linear_acceleration(const uint8_t *data, uint16_t length)
{
    if (length < 10) {
        return;
    }

    int16_t accel_x = read_i16_le(&data[4]);
    int16_t accel_y = read_i16_le(&data[6]);
    int16_t accel_z = read_i16_le(&data[8]);

    latest_linear_accel.x = q_to_float(accel_x, LINEAR_ACCEL_Q);
    latest_linear_accel.y = q_to_float(accel_y, LINEAR_ACCEL_Q);
    latest_linear_accel.z = q_to_float(accel_z, LINEAR_ACCEL_Q);
}

// ============================================================================
// Main Update Loop
// ============================================================================

/**
 * Update - called periodically to read sensor data
 */
void AP_ExternalAHRS_BNO08x::update()
{
    if (!sensor_initialized) {
        // Try to initialize
        if (!init()) {
            return;
        }
    }

    // Try to receive and process packets from sensor
    WITH_SEMAPHORE(dev->get_semaphore());

    // Process multiple packets if available (sensor may batch them)
    for (int i = 0; i < 10; i++) {
        uint16_t packet_length;
        uint32_t timestamp_us;

        if (!shtp_receive(rx_buffer, packet_length, &timestamp_us)) {
            break;
        }

        process_packet(rx_buffer, packet_length);
    }

    // Update shared state if we have new data
    if (have_rotation) {
        WITH_SEMAPHORE(state.sem);

        state.quat = latest_quat;
        state.have_quaternion = true;

        if (have_gyro_data) {
            state.gyro = latest_gyro;
        }

        if (have_accel_data) {
            state.accel = latest_accel;
        }

        // Mark data as consumed
        have_rotation = false;
        have_gyro_data = false;
        have_accel_data = false;
    }
}

// ============================================================================
// Status Functions
// ============================================================================

/**
 * Check if sensor is healthy
 */
bool AP_ExternalAHRS_BNO08x::healthy(void) const
{
    if (!sensor_initialized) {
        return false;
    }

    uint32_t now = AP_HAL::millis();

    // Check if we've received data recently
    return (now - last_packet_ms < SENSOR_TIMEOUT_MS);
}

/**
 * Check if sensor is initialized
 */
bool AP_ExternalAHRS_BNO08x::initialised(void) const
{
    return sensor_initialized;
}

/**
 * Pre-arm check
 */
bool AP_ExternalAHRS_BNO08x::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!sensor_initialized) {
        hal.util->snprintf(failure_msg, failure_msg_len, "BNO08x not initialized");
        return false;
    }

    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "BNO08x unhealthy");
        return false;
    }

    if (!product_id.valid) {
        hal.util->snprintf(failure_msg, failure_msg_len, "BNO08x ID invalid");
        return false;
    }

    return true;
}

/**
 * Get filter status
 */
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
        status.flags.pred_horiz_pos_rel = 0;
        status.flags.pred_horiz_pos_abs = 0;
        status.flags.using_gps = 0;
    }
}

#endif  // AP_EXTERNAL_AHRS_BNO08X_ENABLED
