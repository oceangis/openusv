/**
 * @file wifi_config.h
 * @brief WiFi Configuration System for ESP32-S3 USV (Unmanned Surface Vehicle)
 *
 * Provides WiFi AP/STA mode management, web-based configuration interface,
 * NVS storage for credentials, and MAVLink UDP telemetry forwarding.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Configuration Constants
// ============================================================================

#define WIFI_CONFIG_MAX_SSID_LEN        32
#define WIFI_CONFIG_MAX_PASSWORD_LEN    64
#define WIFI_CONFIG_MAX_STORED_NETWORKS 5
#define WIFI_CONFIG_AP_DEFAULT_CHANNEL  6
#define WIFI_CONFIG_AP_MAX_CONNECTIONS  4

// MAVLink UDP Configuration
#define WIFI_CONFIG_MAVLINK_UDP_PORT    14550
#define WIFI_CONFIG_MAVLINK_BUFFER_SIZE 1024

// HTTP Server Configuration
#define WIFI_CONFIG_HTTP_PORT           80

// Timeouts (in milliseconds)
#define WIFI_CONFIG_CONNECT_TIMEOUT_MS  15000
#define WIFI_CONFIG_SCAN_TIMEOUT_MS     10000
#define WIFI_CONFIG_RECONNECT_DELAY_MS  5000

// ============================================================================
// Type Definitions
// ============================================================================

/**
 * @brief WiFi operation mode
 */
typedef enum {
    WIFI_CONFIG_MODE_OFF = 0,   ///< WiFi disabled
    WIFI_CONFIG_MODE_AP,        ///< Access Point mode (for configuration)
    WIFI_CONFIG_MODE_STA,       ///< Station mode (connect to router)
    WIFI_CONFIG_MODE_APSTA      ///< AP + STA concurrent mode
} wifi_config_mode_t;

/**
 * @brief WiFi connection state
 */
typedef enum {
    WIFI_STATE_DISCONNECTED = 0,    ///< Not connected
    WIFI_STATE_CONNECTING,          ///< Connection in progress
    WIFI_STATE_CONNECTED,           ///< Connected to AP
    WIFI_STATE_CONNECTION_FAILED,   ///< Connection failed
    WIFI_STATE_GOT_IP,              ///< Got IP address
    WIFI_STATE_LOST_IP              ///< Lost IP address
} wifi_connection_state_t;

/**
 * @brief Single WiFi network credentials
 */
typedef struct {
    char ssid[WIFI_CONFIG_MAX_SSID_LEN + 1];
    char password[WIFI_CONFIG_MAX_PASSWORD_LEN + 1];
    int8_t rssi;                    ///< Last known RSSI (-127 if unknown)
    uint8_t priority;               ///< Connection priority (0 = highest)
    bool valid;                     ///< Entry is valid
} wifi_network_entry_t;

/**
 * @brief WiFi scan result entry
 */
typedef struct {
    char ssid[WIFI_CONFIG_MAX_SSID_LEN + 1];
    int8_t rssi;
    uint8_t channel;
    uint8_t authmode;               ///< Authentication mode (wifi_auth_mode_t)
    uint8_t bssid[6];
} wifi_scan_result_t;

/**
 * @brief WiFi configuration structure (stored in NVS)
 */
typedef struct {
    wifi_config_mode_t default_mode;
    char ap_ssid[WIFI_CONFIG_MAX_SSID_LEN + 1];
    char ap_password[WIFI_CONFIG_MAX_PASSWORD_LEN + 1];
    uint8_t ap_channel;
    bool ap_hidden;
    wifi_network_entry_t sta_networks[WIFI_CONFIG_MAX_STORED_NETWORKS];
    bool auto_reconnect;
    bool mavlink_udp_enabled;
    uint16_t mavlink_udp_port;
    char mavlink_udp_target_ip[16];
    uint32_t connect_timeout_ms;
} wifi_system_config_t;

/**
 * @brief WiFi system status
 */
typedef struct {
    wifi_config_mode_t current_mode;
    wifi_connection_state_t sta_state;
    bool ap_active;
    char connected_ssid[WIFI_CONFIG_MAX_SSID_LEN + 1];
    int8_t current_rssi;
    uint8_t current_channel;
    esp_netif_ip_info_t sta_ip_info;
    esp_netif_ip_info_t ap_ip_info;
    uint32_t connected_clients;     ///< Number of clients connected to AP
    uint32_t uptime_seconds;        ///< WiFi uptime
    uint32_t reconnect_count;       ///< Number of reconnection attempts
} wifi_system_status_t;

/**
 * @brief MAVLink UDP statistics
 */
typedef struct {
    uint32_t tx_packets;
    uint32_t rx_packets;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t tx_errors;
    uint32_t rx_errors;
    bool connected;
} wifi_mavlink_stats_t;

/**
 * @brief WiFi event callback type
 */
typedef void (*wifi_event_callback_t)(wifi_connection_state_t state, void *user_data);

/**
 * @brief Config save callback type
 * Called when configuration is modified via Web interface.
 * External system (AP_WiFi_ESP32) can register this to save config to Flash.
 * @param config Pointer to the current configuration
 * @return ESP_OK if saved successfully
 */
typedef esp_err_t (*wifi_config_save_callback_t)(const wifi_system_config_t *config);

// ============================================================================
// System Initialization Functions
// ============================================================================

/**
 * @brief Initialize the WiFi configuration system
 *
 * This is the main entry point called from app_main(). It initializes:
 * - NVS storage
 * - WiFi driver
 * - HTTP server for web configuration
 * - MAVLink UDP forwarder (if enabled)
 *
 * On first boot or if no valid STA configuration exists, starts in AP mode
 * with SSID format "USV_XXXXXX" (last 6 hex digits of MAC address).
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_config_system_init(void);

/**
 * @brief Deinitialize the WiFi configuration system
 *
 * Stops all WiFi operations, HTTP server, and releases resources.
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_config_system_deinit(void);

/**
 * @brief Check if WiFi system is initialized
 *
 * @return true if initialized
 */
bool wifi_config_is_initialized(void);

// ============================================================================
// Mode Control Functions
// ============================================================================

/**
 * @brief Set WiFi operation mode
 *
 * @param mode Target operation mode
 * @return ESP_OK on success
 */
esp_err_t wifi_config_set_mode(wifi_config_mode_t mode);

/**
 * @brief Get current WiFi operation mode
 *
 * @return Current mode
 */
wifi_config_mode_t wifi_config_get_mode(void);

/**
 * @brief Start AP mode for configuration
 *
 * Starts Access Point with current AP settings. SSID defaults to
 * "USV_XXXXXX" format using MAC address.
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_config_start_ap(void);

/**
 * @brief Stop AP mode
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_config_stop_ap(void);

/**
 * @brief Start STA mode and connect
 *
 * Attempts to connect to stored WiFi networks in priority order.
 *
 * @return ESP_OK on success, ESP_ERR_WIFI_NOT_CONNECT if no networks configured
 */
esp_err_t wifi_config_start_sta(void);

/**
 * @brief Stop STA mode
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_config_stop_sta(void);

// ============================================================================
// Configuration Management Functions
// ============================================================================

/**
 * @brief Get current system configuration
 *
 * @param config Pointer to configuration structure to fill
 * @return ESP_OK on success
 */
esp_err_t wifi_config_get_config(wifi_system_config_t *config);

/**
 * @brief Set system configuration
 *
 * @param config Pointer to new configuration
 * @param save_to_nvs If true, save configuration to NVS
 * @return ESP_OK on success
 */
esp_err_t wifi_config_set_config(const wifi_system_config_t *config, bool save_to_nvs);

/**
 * @brief Reset configuration to defaults
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_config_reset_defaults(void);

/**
 * @brief Set AP configuration
 *
 * @param ssid AP SSID (NULL to use auto-generated)
 * @param password AP password (NULL or empty for open network)
 * @param channel WiFi channel (1-13, 0 for auto)
 * @return ESP_OK on success
 */
esp_err_t wifi_config_set_ap_config(const char *ssid, const char *password, uint8_t channel);

// ============================================================================
// Network Management Functions
// ============================================================================

/**
 * @brief Add a WiFi network
 *
 * @param ssid Network SSID
 * @param password Network password
 * @param priority Connection priority (0 = highest)
 * @return ESP_OK on success, ESP_ERR_NO_MEM if storage full
 */
esp_err_t wifi_config_add_network(const char *ssid, const char *password, uint8_t priority);

/**
 * @brief Remove a WiFi network
 *
 * @param ssid Network SSID to remove
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not found
 */
esp_err_t wifi_config_remove_network(const char *ssid);

/**
 * @brief Get stored networks
 *
 * @param networks Array to fill with network entries
 * @param max_count Maximum entries to return
 * @return Number of networks filled
 */
int wifi_config_get_networks(wifi_network_entry_t *networks, int max_count);

/**
 * @brief Clear all stored networks
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_config_clear_networks(void);

// ============================================================================
// Scanning Functions
// ============================================================================

/**
 * @brief Start WiFi scan
 *
 * Scan is asynchronous. Use wifi_config_get_scan_results() to get results.
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_config_start_scan(void);

/**
 * @brief Get scan results
 *
 * @param results Array to fill with scan results
 * @param max_count Maximum results to return
 * @return Number of results filled, or -1 if scan in progress
 */
int wifi_config_get_scan_results(wifi_scan_result_t *results, int max_count);

/**
 * @brief Check if scan is in progress
 *
 * @return true if scanning
 */
bool wifi_config_is_scanning(void);

// ============================================================================
// Connection Management Functions
// ============================================================================

/**
 * @brief Connect to a specific network
 *
 * @param ssid Network SSID
 * @param password Network password (can be NULL for open networks)
 * @return ESP_OK on success
 */
esp_err_t wifi_config_connect(const char *ssid, const char *password);

/**
 * @brief Disconnect from current network
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_config_disconnect(void);

/**
 * @brief Get current connection state
 *
 * @return Current state
 */
wifi_connection_state_t wifi_config_get_connection_state(void);

/**
 * @brief Get system status
 *
 * @param status Pointer to status structure to fill
 * @return ESP_OK on success
 */
esp_err_t wifi_config_get_status(wifi_system_status_t *status);

/**
 * @brief Register event callback
 *
 * @param callback Callback function
 * @param user_data User data passed to callback
 * @return ESP_OK on success
 */
esp_err_t wifi_config_register_callback(wifi_event_callback_t callback, void *user_data);

/**
 * @brief Register config save callback
 *
 * When registered, config changes will be saved via this callback instead of NVS.
 * This allows AP_WiFi_ESP32 to save config to ArduPilot's Flash storage.
 *
 * @param callback Callback function to save config
 * @return ESP_OK on success
 */
esp_err_t wifi_config_register_save_callback(wifi_config_save_callback_t callback);

/**
 * @brief Check if external storage callback is registered
 *
 * @return true if using external storage (AP_Param Flash)
 */
bool wifi_config_use_external_storage(void);

// ============================================================================
// MAVLink UDP Functions
// ============================================================================

/**
 * @brief Enable MAVLink UDP forwarding
 *
 * @param enable true to enable, false to disable
 * @param target_ip Target IP address (NULL for broadcast)
 * @param port UDP port (0 for default 14550)
 * @return ESP_OK on success
 */
esp_err_t wifi_config_mavlink_enable(bool enable, const char *target_ip, uint16_t port);

/**
 * @brief Send MAVLink data over UDP
 *
 * @param data Data to send
 * @param len Data length
 * @return Number of bytes sent, or -1 on error
 */
int wifi_config_mavlink_send(const uint8_t *data, size_t len);

/**
 * @brief Receive MAVLink data from UDP
 *
 * @param data Buffer to receive data
 * @param max_len Maximum bytes to receive
 * @return Number of bytes received, or -1 on error
 */
int wifi_config_mavlink_receive(uint8_t *data, size_t max_len);

/**
 * @brief Check if MAVLink data is available
 *
 * @return Number of bytes available
 */
int wifi_config_mavlink_available(void);

/**
 * @brief Get MAVLink UDP statistics
 *
 * @param stats Pointer to stats structure
 * @return ESP_OK on success
 */
esp_err_t wifi_config_mavlink_get_stats(wifi_mavlink_stats_t *stats);

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Get device MAC address
 *
 * @param mac Buffer for MAC address (6 bytes)
 * @return ESP_OK on success
 */
esp_err_t wifi_config_get_mac(uint8_t *mac);

/**
 * @brief Get auto-generated AP SSID
 *
 * Format: "USV_XXXXXX" where XXXXXX is last 6 hex digits of MAC
 *
 * @param ssid Buffer for SSID (minimum 16 bytes)
 * @return ESP_OK on success
 */
esp_err_t wifi_config_get_default_ap_ssid(char *ssid);

/**
 * @brief Convert auth mode to string
 *
 * @param authmode Authentication mode value
 * @return String representation
 */
const char *wifi_config_authmode_str(uint8_t authmode);

#ifdef __cplusplus
}
#endif
