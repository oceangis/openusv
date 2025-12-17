/**
 * @file AP_WiFi_ESP32.h
 * @brief ArduPilot WiFi Parameter Integration for ESP32-S3
 *
 * This class integrates WiFi configuration with ArduPilot's AP_Param system,
 * allowing WiFi settings to be configured via MissionPlanner/GCS while maintaining
 * synchronization with the underlying wifi_config NVS storage.
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_WIFI_ESP32_ENABLED
#define AP_WIFI_ESP32_ENABLED 1
#endif

#if AP_WIFI_ESP32_ENABLED

#include <AP_Param/AP_Param.h>

// Forward declaration of C functions from wifi_config
extern "C" {
    #include "wifi_config.h"
}

/**
 * @brief WiFi configuration class for ArduPilot parameter system
 *
 * Provides WIFI_ prefixed parameters that can be configured through GCS.
 * Automatically synchronizes with the wifi_config NVS storage system.
 */
class AP_WiFi_ESP32 {
public:
    AP_WiFi_ESP32();

    // Do not allow copies
    CLASS_NO_COPY(AP_WiFi_ESP32);

    // Parameter table
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Initialize WiFi system and sync parameters
     */
    void init();

    /**
     * @brief Update method called from main loop
     * Handles parameter-to-config synchronization
     */
    void update();

    /**
     * @brief Get singleton instance
     */
    static AP_WiFi_ESP32 *get_singleton() { return _singleton; }

    /**
     * @brief Sync ArduPilot parameters to wifi_config system
     */
    void sync_params_to_config();

    /**
     * @brief Sync wifi_config system to ArduPilot parameters
     */
    void sync_config_to_params();

    /**
     * @brief Check if WiFi is enabled
     */
    bool is_enabled() const { return _enable.get() > 0; }

    /**
     * @brief Get current WiFi mode
     */
    int8_t get_mode() const { return _mode.get(); }

    /**
     * @brief Get WiFi connection status
     * @return true if connected with IP
     */
    bool is_connected() const;

    /**
     * @brief Get current RSSI in dBm
     */
    int8_t get_rssi() const;

    /**
     * @brief Get current IP address as string
     */
    const char* get_ip_address() const;

    /**
     * @brief Update AP_Param from wifi_config (called by C save callback)
     * @param config Configuration from Web interface
     */
    void update_params_from_config(const wifi_system_config_t *config);

    // WiFi mode enumeration matching wifi_config_mode_t
    enum WiFiMode : int8_t {
        MODE_OFF = 0,
        MODE_AP = 1,
        MODE_STA = 2,
        MODE_APSTA = 3
    };

private:
    static AP_WiFi_ESP32 *_singleton;

    // Primary enable parameter
    AP_Int8 _enable;            // WIFI_ENABLE: 0=disabled, 1=enabled

    // Mode selection
    AP_Int8 _mode;              // WIFI_MODE: 0=OFF, 1=AP, 2=STA, 3=APSTA

    // AP mode configuration
    AP_Int8 _ap_channel;        // WIFI_AP_CHAN: WiFi channel 1-13
    AP_Int8 _ap_hidden;         // WIFI_AP_HIDE: 0=visible, 1=hidden

    // MAVLink UDP configuration
    AP_Int8 _mavlink_enable;    // WIFI_MAV_EN: 0=disabled, 1=enabled
    AP_Int16 _mavlink_port;     // WIFI_MAV_PORT: UDP port (default 14550)

    // Connection settings
    AP_Int16 _connect_timeout;  // WIFI_CON_TMO: Connection timeout in seconds
    AP_Int8 _auto_reconnect;    // WIFI_AUTOCON: 0=disabled, 1=enabled

    // Status tracking (not stored as parameters)
    uint32_t _last_sync_ms;
    bool _initialized;
    char _ip_addr_str[16];

    // Parameter change detection
    int8_t _last_enable;
    int8_t _last_mode;
    int8_t _last_ap_channel;
    int8_t _last_mavlink_enable;
    int16_t _last_mavlink_port;

    /**
     * @brief Check if any AP_Param parameters have changed
     */
    bool params_changed() const;

    /**
     * @brief Check if wifi_config memory has changed (from Web interface)
     */
    bool wifi_config_memory_changed();

    /**
     * @brief Apply current AP_Param parameters to wifi_config memory
     */
    void apply_params_to_wifi_config();

    /**
     * @brief Config snapshot for change detection
     */
    wifi_system_config_t _nvs_snapshot;
};

#endif // AP_WIFI_ESP32_ENABLED
