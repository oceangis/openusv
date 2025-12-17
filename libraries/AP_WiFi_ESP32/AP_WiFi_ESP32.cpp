/**
 * @file AP_WiFi_ESP32.cpp
 * @brief ArduPilot WiFi Parameter Integration Implementation
 *
 * DESIGN: AP_Param (Flash partition 0x45) is the Single Source of Truth
 * - All WiFi configuration is stored in ArduPilot's parameter system
 * - wifi_config operates on in-memory copy only (no NVS)
 * - Web interface changes are routed through AP_Param via save callback
 *
 * Data Flow:
 * - MissionPlanner -> AP_Param (Flash) -> wifi_config (memory)
 * - Web interface -> save callback -> AP_Param (Flash) -> wifi_config (memory)
 */

#include "AP_WiFi_ESP32.h"

#if AP_WIFI_ESP32_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <string.h>
#include <stdio.h>

// ESP-IDF network macros
#include "lwip/ip4_addr.h"

extern const AP_HAL::HAL& hal;

AP_WiFi_ESP32 *AP_WiFi_ESP32::_singleton = nullptr;

// ============================================================================
// C-linkage save callback for wifi_config
// ============================================================================

/**
 * @brief Static C callback invoked when Web interface saves config
 *
 * This bridges C (wifi_config) to C++ (AP_WiFi_ESP32).
 * Updates AP_Param parameters which then get saved to Flash.
 */
extern "C" esp_err_t wifi_config_save_to_ap_param(const wifi_system_config_t *config)
{
    AP_WiFi_ESP32 *wifi = AP_WiFi_ESP32::get_singleton();
    if (!wifi || !config) {
        return ESP_ERR_INVALID_STATE;
    }

    // Update AP_Param from wifi_config (Web interface changes)
    wifi->update_params_from_config(config);

    return ESP_OK;
}

// ============================================================================
// Parameter table definition
// ============================================================================

// @Group: WIFI_
// @Path: AP_WiFi_ESP32.cpp
const AP_Param::GroupInfo AP_WiFi_ESP32::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: WiFi Enable
    // @Description: Enable or disable WiFi functionality
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_WiFi_ESP32, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: MODE
    // @DisplayName: WiFi Mode
    // @Description: WiFi operation mode. AP creates an access point, STA connects to router, APSTA does both.
    // @Values: 0:OFF,1:AP,2:STA,3:APSTA
    // @User: Standard
    AP_GROUPINFO("MODE", 2, AP_WiFi_ESP32, _mode, 3),

    // @Param: AP_CHAN
    // @DisplayName: AP Channel
    // @Description: WiFi channel for Access Point mode (1-13)
    // @Range: 1 13
    // @User: Advanced
    AP_GROUPINFO("AP_CHAN", 3, AP_WiFi_ESP32, _ap_channel, 6),

    // @Param: AP_HIDE
    // @DisplayName: AP Hidden
    // @Description: Hide AP SSID from scans
    // @Values: 0:Visible,1:Hidden
    // @User: Advanced
    AP_GROUPINFO("AP_HIDE", 4, AP_WiFi_ESP32, _ap_hidden, 0),

    // @Param: MAV_EN
    // @DisplayName: MAVLink UDP Enable
    // @Description: Enable MAVLink UDP forwarding over WiFi
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("MAV_EN", 5, AP_WiFi_ESP32, _mavlink_enable, 1),

    // @Param: MAV_PORT
    // @DisplayName: MAVLink UDP Port
    // @Description: UDP port for MAVLink communication (default 14550)
    // @Range: 1024 65535
    // @User: Advanced
    AP_GROUPINFO("MAV_PORT", 6, AP_WiFi_ESP32, _mavlink_port, 14550),

    // @Param: CON_TMO
    // @DisplayName: Connection Timeout
    // @Description: Timeout for WiFi STA connection in seconds
    // @Range: 5 60
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("CON_TMO", 7, AP_WiFi_ESP32, _connect_timeout, 15),

    // @Param: AUTOCON
    // @DisplayName: Auto Reconnect
    // @Description: Automatically reconnect to WiFi if connection is lost
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("AUTOCON", 8, AP_WiFi_ESP32, _auto_reconnect, 1),

    AP_GROUPEND
};

// ============================================================================
// Constructor
// ============================================================================

AP_WiFi_ESP32::AP_WiFi_ESP32()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_WiFi_ESP32 must be singleton");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    _last_sync_ms = 0;
    _initialized = false;
    memset(_ip_addr_str, 0, sizeof(_ip_addr_str));

    // Initialize change detection
    _last_enable = -1;
    _last_mode = -1;
    _last_ap_channel = -1;
    _last_mavlink_enable = -1;
    _last_mavlink_port = -1;

    // Initialize config snapshot
    memset(&_nvs_snapshot, 0, sizeof(_nvs_snapshot));
}

// ============================================================================
// Initialization
// ============================================================================

void AP_WiFi_ESP32::init()
{
    if (_initialized) {
        return;
    }

    if (!is_enabled()) {
        return;
    }

    // Register our save callback so Web interface changes come through AP_Param
    wifi_config_register_save_callback(wifi_config_save_to_ap_param);

    // Apply current AP_Param values to wifi_config memory
    // This is the authoritative source (Flash partition 0x45)
    apply_params_to_wifi_config();

    // Store initial parameter values for change detection
    _last_enable = _enable.get();
    _last_mode = _mode.get();
    _last_ap_channel = _ap_channel.get();
    _last_mavlink_enable = _mavlink_enable.get();
    _last_mavlink_port = _mavlink_port.get();

    // Take initial snapshot for Web interface change detection
    wifi_config_get_config(&_nvs_snapshot);

    _initialized = true;

    hal.console->printf("AP_WiFi_ESP32: initialized, mode=%d (AP_Param is source of truth)\n", _mode.get());
}

// ============================================================================
// Update loop
// ============================================================================

void AP_WiFi_ESP32::update()
{
    if (!_initialized) {
        init();
        return;
    }

    // Check for parameter changes (from MissionPlanner)
    if (params_changed()) {
        // MissionPlanner modified parameters -> apply to wifi_config memory
        apply_params_to_wifi_config();
        hal.console->printf("AP_WiFi_ESP32: AP_Param->wifi_config sync\n");
    }

    // Periodically check for config changes from Web interface
    // The save callback handles saving to AP_Param, but we need to detect
    // changes to update runtime behavior
    uint32_t now = AP_HAL::millis();
    if (now - _last_sync_ms > 1000) {
        if (wifi_config_memory_changed()) {
            // Web interface modified config (already saved via callback)
            // Just update our change detection state
            hal.console->printf("AP_WiFi_ESP32: Web config change detected\n");
        }
        _last_sync_ms = now;
    }
}

// ============================================================================
// Parameter change detection
// ============================================================================

bool AP_WiFi_ESP32::params_changed() const
{
    return (_enable.get() != _last_enable) ||
           (_mode.get() != _last_mode) ||
           (_ap_channel.get() != _last_ap_channel) ||
           (_mavlink_enable.get() != _last_mavlink_enable) ||
           (_mavlink_port.get() != _last_mavlink_port);
}

bool AP_WiFi_ESP32::wifi_config_memory_changed()
{
    wifi_system_config_t current;
    if (wifi_config_get_config(&current) != ESP_OK) {
        return false;
    }

    // Compare current in-memory config with snapshot
    bool changed = (current.default_mode != _nvs_snapshot.default_mode) ||
                   (current.ap_channel != _nvs_snapshot.ap_channel) ||
                   (current.ap_hidden != _nvs_snapshot.ap_hidden) ||
                   (current.mavlink_udp_enabled != _nvs_snapshot.mavlink_udp_enabled) ||
                   (current.mavlink_udp_port != _nvs_snapshot.mavlink_udp_port) ||
                   (current.connect_timeout_ms != _nvs_snapshot.connect_timeout_ms) ||
                   (current.auto_reconnect != _nvs_snapshot.auto_reconnect);

    if (changed) {
        // Update snapshot
        memcpy(&_nvs_snapshot, &current, sizeof(_nvs_snapshot));
    }

    return changed;
}

// ============================================================================
// Apply AP_Param to wifi_config memory
// ============================================================================

void AP_WiFi_ESP32::apply_params_to_wifi_config()
{
    // Get current config from wifi_config memory
    wifi_system_config_t config;
    if (wifi_config_get_config(&config) != ESP_OK) {
        return;
    }

    bool config_changed = false;

    // Update mode
    if (_mode.get() != (int8_t)config.default_mode) {
        config.default_mode = (wifi_config_mode_t)_mode.get();
        config_changed = true;
    }

    // Update AP channel
    if (_ap_channel.get() != config.ap_channel) {
        config.ap_channel = _ap_channel.get();
        config_changed = true;
    }

    // Update AP hidden
    if (_ap_hidden.get() != (config.ap_hidden ? 1 : 0)) {
        config.ap_hidden = _ap_hidden.get() != 0;
        config_changed = true;
    }

    // Update MAVLink settings
    if (_mavlink_enable.get() != (config.mavlink_udp_enabled ? 1 : 0)) {
        config.mavlink_udp_enabled = _mavlink_enable.get() != 0;
        config_changed = true;
    }

    if (_mavlink_port.get() != config.mavlink_udp_port) {
        config.mavlink_udp_port = _mavlink_port.get();
        config_changed = true;
    }

    // Update connection timeout
    uint32_t timeout_ms = _connect_timeout.get() * 1000;
    if (timeout_ms != config.connect_timeout_ms) {
        config.connect_timeout_ms = timeout_ms;
        config_changed = true;
    }

    // Update auto reconnect
    if (_auto_reconnect.get() != (config.auto_reconnect ? 1 : 0)) {
        config.auto_reconnect = _auto_reconnect.get() != 0;
        config_changed = true;
    }

    // Update wifi_config memory (save_to_nvs=false since we're the source of truth)
    if (config_changed) {
        wifi_config_set_config(&config, false);
    }

    // Handle runtime mode change
    if (_mode.get() != _last_mode) {
        wifi_config_set_mode((wifi_config_mode_t)_mode.get());
    }

    // Handle MAVLink enable/disable at runtime
    if (_mavlink_enable.get() != _last_mavlink_enable ||
        _mavlink_port.get() != _last_mavlink_port) {
        wifi_config_mavlink_enable(
            _mavlink_enable.get() != 0,
            config.mavlink_udp_target_ip,
            _mavlink_port.get()
        );
    }

    // Update change detection values
    _last_enable = _enable.get();
    _last_mode = _mode.get();
    _last_ap_channel = _ap_channel.get();
    _last_mavlink_enable = _mavlink_enable.get();
    _last_mavlink_port = _mavlink_port.get();

    // Update snapshot
    memcpy(&_nvs_snapshot, &config, sizeof(_nvs_snapshot));
}

// ============================================================================
// Update AP_Param from wifi_config (called by save callback)
// ============================================================================

void AP_WiFi_ESP32::update_params_from_config(const wifi_system_config_t *config)
{
    if (!config) {
        return;
    }

    // Update AP_Param parameters from wifi_config
    // set_and_save_ifchanged() will save to Flash (partition 0x45) if changed
    _mode.set_and_save_ifchanged((int8_t)config->default_mode);
    _ap_channel.set_and_save_ifchanged((int8_t)config->ap_channel);
    _ap_hidden.set_and_save_ifchanged(config->ap_hidden ? 1 : 0);
    _mavlink_enable.set_and_save_ifchanged(config->mavlink_udp_enabled ? 1 : 0);
    _mavlink_port.set_and_save_ifchanged((int16_t)config->mavlink_udp_port);
    _connect_timeout.set_and_save_ifchanged((int16_t)(config->connect_timeout_ms / 1000));
    _auto_reconnect.set_and_save_ifchanged(config->auto_reconnect ? 1 : 0);

    // Update change detection values
    _last_mode = _mode.get();
    _last_ap_channel = _ap_channel.get();
    _last_mavlink_enable = _mavlink_enable.get();
    _last_mavlink_port = _mavlink_port.get();

    // Update snapshot
    memcpy(&_nvs_snapshot, config, sizeof(_nvs_snapshot));

    hal.console->printf("AP_WiFi_ESP32: Web->AP_Param saved to Flash\n");
}

// ============================================================================
// Legacy sync functions (for compatibility)
// ============================================================================

void AP_WiFi_ESP32::sync_params_to_config()
{
    apply_params_to_wifi_config();
}

void AP_WiFi_ESP32::sync_config_to_params()
{
    wifi_system_config_t config;
    if (wifi_config_get_config(&config) != ESP_OK) {
        return;
    }
    update_params_from_config(&config);
}

// ============================================================================
// Status functions
// ============================================================================

bool AP_WiFi_ESP32::is_connected() const
{
    wifi_system_status_t status;
    if (wifi_config_get_status(&status) != ESP_OK) {
        return false;
    }
    return status.sta_state == WIFI_STATE_GOT_IP;
}

int8_t AP_WiFi_ESP32::get_rssi() const
{
    wifi_system_status_t status;
    if (wifi_config_get_status(&status) != ESP_OK) {
        return -127;
    }
    return status.current_rssi;
}

const char* AP_WiFi_ESP32::get_ip_address() const
{
    static char ip_str[16];
    wifi_system_status_t status;
    if (wifi_config_get_status(&status) != ESP_OK) {
        return "0.0.0.0";
    }

    snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&status.sta_ip_info.ip));
    return ip_str;
}

#endif // AP_WIFI_ESP32_ENABLED
