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

  Based on deep analysis of WORKING Arduino code and Adafruit BNO08x library.

  Critical fixes in v2.1:
  1. REMOVED INT pin polling for I2C mode (INT is only for SPI in Adafruit lib!)
  2. FIXED I2C chunked read logic to match Adafruit i2chal_read() exactly
  3. Continuous polling (5ms delay) matching Arduino test code pattern
  4. Correct sensor report parsing and data flow

  Key implementation details:
  1. 500ms startup delay before first communication (critical for BNO08x boot)
  2. Multiple retry mechanism (up to 5 attempts) for soft reset
  3. Proper SHTP packet handling matching Adafruit implementation
  4. wasReset() detection for automatic reconfiguration
  5. Thread-based I2C operations to prevent main loop blocking

  Reference implementations:
  - Adafruit BNO08x Library (Hillcrest SH2 Driver)
  - SparkFun BNO08x Library
  - BNO08x Datasheet and SH-2 Reference Manual
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_BNO08X_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/Semaphores.h>

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

    // Thread-based update
    void update_thread();
    HAL_Semaphore data_sem;
    bool thread_started;

    // Initialization - MUST be in thread context
    bool init_sensor();
    bool probe_device(uint8_t bus, uint8_t addr);

    // Soft reset with retries (matching Adafruit i2chal_open)
    bool soft_reset_with_retry(uint8_t max_attempts);
    bool wait_for_reset_complete();
    bool get_product_ids();
    bool configure_sensors();

    // INT pin handling (NOT USED for I2C mode - only for SPI)
    // Kept for potential future SPI support
    bool init_int_pin();
    bool data_ready();
    bool wait_for_int(uint32_t timeout_ms);

    // SHTP protocol handling - matching Adafruit implementation
    bool shtp_send_packet(uint8_t channel, const uint8_t *data, uint16_t length);
    bool shtp_receive_packet();

    // Low-level I2C - exact match with Adafruit i2chal_read/write
    int hal_read(uint8_t *pBuffer, unsigned len);
    int hal_write(const uint8_t *pBuffer, unsigned len);

    // Sensor configuration
    bool enable_report(uint8_t sensor_id, uint32_t interval_us);
    bool enable_rotation_vector(uint32_t interval_us);
    bool enable_gyro(uint32_t interval_us);
    bool enable_accelerometer(uint32_t interval_us);

    // Data parsing
    void process_packet();
    void parse_sensor_reports(const uint8_t *data, uint16_t length);
    void parse_rotation_vector(const uint8_t *data, uint16_t length);
    void parse_game_rotation_vector(const uint8_t *data, uint16_t length);
    void parse_gyro_calibrated(const uint8_t *data, uint16_t length);
    void parse_accelerometer(const uint8_t *data, uint16_t length);
    void parse_linear_acceleration(const uint8_t *data, uint16_t length);
    void parse_command_response(const uint8_t *data, uint16_t length);

    // Check if reset occurred (like Adafruit wasReset())
    bool check_and_handle_reset();

    // Q point conversion utilities (from Hillcrest)
    static float q_to_float(int16_t fixed_point, uint8_t q_point);
    static int16_t read_i16_le(const uint8_t *buffer);
    static uint16_t read_u16_le(const uint8_t *buffer);
    static uint32_t read_u32_le(const uint8_t *buffer);

    // SHTP constants
    static constexpr uint16_t MAX_PACKET_SIZE = 512;
    static constexpr uint16_t MAX_I2C_BUFFER = 128;  // ESP32 I2C buffer limit
    static constexpr uint8_t SHTP_HDR_LEN = 4;

    // Packet buffers
    uint8_t rx_buffer[MAX_PACKET_SIZE];
    uint8_t tx_buffer[MAX_PACKET_SIZE];
    uint16_t rx_packet_len;  // Current received packet length
    uint8_t rx_channel;      // Current received packet channel
    uint8_t rx_sequence;     // Current received packet sequence

    // BNO08x I2C addresses
    static constexpr uint8_t BNO08X_I2C_ADDR = 0x4A;
    static constexpr uint8_t BNO08X_I2C_ADDR_ALT = 0x4B;

    // BNO08x INT pin - GPIO19 (active low when data ready)
    static constexpr uint8_t BNO08X_INT_PIN = 19;

    // SHTP Channel IDs (from Hillcrest SHTP specification)
    static constexpr uint8_t SHTP_CHAN_COMMAND = 0;      // SHTP command/advertisement
    static constexpr uint8_t SHTP_CHAN_EXECUTABLE = 1;   // Executable/device control
    static constexpr uint8_t SHTP_CHAN_CONTROL = 2;      // Sensorhub control (Set Feature)
    static constexpr uint8_t SHTP_CHAN_REPORTS = 3;      // Sensor input reports
    static constexpr uint8_t SHTP_CHAN_WAKE_REPORTS = 4; // Wake sensor input reports
    static constexpr uint8_t SHTP_CHAN_GYRO_RV = 5;      // Gyro rotation vector

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

    // Executable channel response codes
    static constexpr uint8_t EXECUTABLE_RESET_COMPLETE = 1;

    // SH2 Command IDs
    static constexpr uint8_t SH2_CMD_INITIALIZE = 4;
    static constexpr uint8_t SH2_INIT_UNSOLICITED = 0x80;
    static constexpr uint8_t SH2_INIT_SYSTEM = 1;

    // Q point values for different sensors (from SH-2 Reference Manual)
    static constexpr uint8_t ROTATION_VECTOR_Q = 14;
    static constexpr uint8_t ROTATION_ACCURACY_Q = 12;
    static constexpr uint8_t ACCELEROMETER_Q = 8;
    static constexpr uint8_t LINEAR_ACCEL_Q = 8;
    static constexpr uint8_t GYROSCOPE_Q = 9;
    static constexpr uint8_t MAGNETOMETER_Q = 4;
    static constexpr uint8_t ANGULAR_VELOCITY_Q = 10;
    static constexpr uint8_t GRAVITY_Q = 8;

    // Sequence numbers per channel (like Adafruit)
    uint8_t seq_num[6];

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

    // Reset detection (like Adafruit wasReset)
    volatile bool reset_occurred;

    // State tracking
    volatile bool sensor_initialized;
    volatile uint32_t last_packet_ms;
    volatile uint32_t last_rotation_ms;
    volatile uint32_t last_gyro_ms;
    volatile uint32_t last_accel_ms;

    // Sensor data (protected by data_sem)
    Vector3f latest_gyro;
    Vector3f latest_accel;
    Vector3f latest_linear_accel;
    Quaternion latest_quat;
    float quat_accuracy_rad;
    uint8_t quat_status;

    volatile bool have_rotation;
    volatile bool have_gyro_data;
    volatile bool have_accel_data;

    // Statistics
    uint32_t packet_count;
    uint32_t error_count;
    uint32_t reset_count;
    uint32_t consecutive_failures;      // I2C 连续失败计数
    uint32_t last_stats_ms;             // 上次输出统计的时间
    uint32_t reinit_count;              // 重新初始化次数

    // Timing constants
    static constexpr uint32_t SENSOR_TIMEOUT_MS = 500;
    static constexpr uint32_t RESET_TIMEOUT_MS = 2000;
    static constexpr uint32_t STARTUP_DELAY_MS = 500;  // CRITICAL: Wait for BNO08x to boot
    static constexpr uint32_t SOFTRESET_DELAY_MS = 300;  // Delay after soft reset
    static constexpr uint32_t DEFAULT_REPORT_INTERVAL_US = 10000;  // 100Hz (10ms interval = 10000us)
    static constexpr uint8_t MAX_INIT_ATTEMPTS = 5;  // Match Arduino code retry count
    static constexpr uint32_t MAX_CONSECUTIVE_FAILURES = 50;   // 连续失败50次后重新初始化
    static constexpr uint32_t STATS_INTERVAL_MS = 10000;       // 每10秒输出一次统计
};

#endif  // AP_EXTERNAL_AHRS_BNO08X_ENABLED
