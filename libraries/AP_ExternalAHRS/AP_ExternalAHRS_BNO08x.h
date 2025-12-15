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

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_BNO08X_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_ExternalAHRS_BNO08x : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_BNO08x(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // Get model/type name
    const char* get_name() const override { return "BNO08x"; }

    // Accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;

    // Check for new data
    void update() override;

    // Get I2C port number (not applicable for I2C device)
    int8_t get_port(void) const override { return -1; }

    uint8_t num_gps_sensors(void) const override { return 0; }

private:
    // BNO08x I2C device
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    // Initialization
    bool init();
    bool soft_reset();
    bool wait_for_reset_complete();
    bool get_product_ids();
    bool configure_sensor();

    // SHTP protocol handling
    bool shtp_receive(uint8_t *packet, uint16_t &length, uint32_t *timestamp_us);
    bool shtp_send(uint8_t channel, const uint8_t *data, uint16_t length);

    // I2C helpers - following Adafruit/SparkFun pattern
    bool i2c_read_packet(uint8_t *buffer, uint16_t max_len, uint16_t &actual_len);
    bool i2c_write_packet(const uint8_t *buffer, uint16_t len);

    // Sensor configuration
    bool enable_report(uint8_t sensor_id, uint32_t report_interval_us);
    bool enable_rotation_vector(uint32_t report_interval_us);
    bool enable_gyro(uint32_t report_interval_us);
    bool enable_accelerometer(uint32_t report_interval_us);
    bool enable_linear_acceleration(uint32_t report_interval_us);

    // Data parsing
    void process_packet(uint8_t *packet, uint16_t length);
    void parse_sensor_reports(uint8_t *data, uint16_t length);
    void parse_rotation_vector(const uint8_t *data, uint16_t length);
    void parse_game_rotation_vector(const uint8_t *data, uint16_t length);
    void parse_gyro_calibrated(const uint8_t *data, uint16_t length);
    void parse_accelerometer(const uint8_t *data, uint16_t length);
    void parse_linear_acceleration(const uint8_t *data, uint16_t length);

    // Q point conversion utilities (from Hillcrest)
    static float q_to_float(int16_t fixed_point, uint8_t q_point);
    static int16_t read_i16_le(const uint8_t *buffer);
    static uint16_t read_u16_le(const uint8_t *buffer);
    static uint32_t read_u32_le(const uint8_t *buffer);

    // SHTP packet buffer
    static constexpr uint16_t MAX_PACKET_SIZE = 512;
    static constexpr uint16_t MAX_I2C_TRANSFER = 256;  // Typical I2C buffer limit
    uint8_t rx_buffer[MAX_PACKET_SIZE];
    uint8_t tx_buffer[MAX_PACKET_SIZE];

    // BNO08x I2C addresses
    static constexpr uint8_t BNO08X_I2C_ADDR = 0x4A;
    static constexpr uint8_t BNO08X_I2C_ADDR_ALT = 0x4B;

    // SHTP Channel IDs (from Hillcrest SHTP specification)
    static constexpr uint8_t SHTP_CHAN_COMMAND = 0;      // SHTP command/advertisement
    static constexpr uint8_t SHTP_CHAN_EXECUTABLE = 1;   // Executable/device control
    static constexpr uint8_t SHTP_CHAN_CONTROL = 2;      // Sensorhub control (Set Feature)
    static constexpr uint8_t SHTP_CHAN_REPORTS = 3;      // Sensor input reports

    // SHTP Header size
    static constexpr uint8_t SHTP_HDR_LEN = 4;

    // Sensor Report IDs (from SH-2 Reference Manual)
    static constexpr uint8_t SENSOR_ACCELEROMETER = 0x01;
    static constexpr uint8_t SENSOR_GYROSCOPE_CALIBRATED = 0x02;
    static constexpr uint8_t SENSOR_MAGNETIC_FIELD = 0x03;
    static constexpr uint8_t SENSOR_LINEAR_ACCELERATION = 0x04;
    static constexpr uint8_t SENSOR_ROTATION_VECTOR = 0x05;
    static constexpr uint8_t SENSOR_GRAVITY = 0x06;
    static constexpr uint8_t SENSOR_GYROSCOPE_UNCALIBRATED = 0x07;
    static constexpr uint8_t SENSOR_GAME_ROTATION_VECTOR = 0x08;
    static constexpr uint8_t SENSOR_GEOMAG_ROTATION_VECTOR = 0x09;
    static constexpr uint8_t SENSOR_RAW_ACCELEROMETER = 0x14;
    static constexpr uint8_t SENSOR_RAW_GYROSCOPE = 0x15;
    static constexpr uint8_t SENSOR_RAW_MAGNETOMETER = 0x16;
    static constexpr uint8_t SENSOR_GYRO_INTEGRATED_RV = 0x2A;

    // SH2 Command/Response Report IDs
    static constexpr uint8_t SENSORHUB_COMMAND_RESP = 0xF1;
    static constexpr uint8_t SENSORHUB_COMMAND_REQ = 0xF2;
    static constexpr uint8_t SENSORHUB_FRS_READ_RESP = 0xF3;
    static constexpr uint8_t SENSORHUB_FRS_READ_REQ = 0xF4;
    static constexpr uint8_t SENSORHUB_PROD_ID_RESP = 0xF8;
    static constexpr uint8_t SENSORHUB_PROD_ID_REQ = 0xF9;
    static constexpr uint8_t SENSORHUB_BASE_TIMESTAMP = 0xFB;
    static constexpr uint8_t SENSORHUB_GET_FEATURE_RESP = 0xFC;
    static constexpr uint8_t SENSORHUB_SET_FEATURE_CMD = 0xFD;
    static constexpr uint8_t SENSORHUB_GET_FEATURE_REQ = 0xFE;

    // Executable channel commands
    static constexpr uint8_t EXECUTABLE_CMD_RESET = 1;
    static constexpr uint8_t EXECUTABLE_CMD_ON = 2;
    static constexpr uint8_t EXECUTABLE_CMD_SLEEP = 3;
    static constexpr uint8_t EXECUTABLE_RESET_COMPLETE = 1;

    // Q point values for different sensors (from SH-2 Reference Manual)
    static constexpr uint8_t ROTATION_VECTOR_Q = 14;
    static constexpr uint8_t ROTATION_ACCURACY_Q = 12;
    static constexpr uint8_t ACCELEROMETER_Q = 8;
    static constexpr uint8_t LINEAR_ACCEL_Q = 8;
    static constexpr uint8_t GYROSCOPE_Q = 9;
    static constexpr uint8_t MAGNETOMETER_Q = 4;
    static constexpr uint8_t ANGULAR_VELOCITY_Q = 10;
    static constexpr uint8_t GRAVITY_Q = 8;

    // Sequence numbers per channel
    uint8_t seq_num[6];  // One per channel (up to 6 channels)

    // Product ID info
    struct {
        uint8_t reset_cause;
        uint8_t sw_version_major;
        uint8_t sw_version_minor;
        uint32_t sw_part_number;
        uint32_t sw_build_number;
        uint16_t sw_version_patch;
        bool valid;
    } product_id;

    // State tracking
    bool sensor_initialized;
    bool reset_complete;
    uint32_t last_packet_ms;
    uint32_t last_rotation_ms;
    uint32_t last_gyro_ms;
    uint32_t last_accel_ms;
    uint64_t sensor_timestamp_us;  // Timestamp from sensor

    // Sensor data (temporary storage before copying to state)
    Vector3f latest_gyro;
    Vector3f latest_accel;
    Vector3f latest_linear_accel;
    Quaternion latest_quat;
    float quat_accuracy_rad;
    uint8_t quat_status;

    bool have_rotation;
    bool have_gyro_data;
    bool have_accel_data;

    // Timing constants
    static constexpr uint32_t SENSOR_TIMEOUT_MS = 500;
    static constexpr uint32_t RESET_TIMEOUT_MS = 1000;
    static constexpr uint32_t DEFAULT_REPORT_INTERVAL_US = 10000; // 100Hz
};

#endif  // AP_EXTERNAL_AHRS_BNO08X_ENABLED
