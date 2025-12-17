/**
 * @file wifi_manager.c
 * @brief WiFi Manager - Advanced WiFi management features
 *
 * Provides enhanced WiFi management including:
 * - Multi-network connection management
 * - Signal quality monitoring
 * - Network roaming
 * - Connection health monitoring
 * - Background scanning for better networks
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "wifi_config.h"

// ============================================================================
// Constants and Definitions
// ============================================================================

static const char *TAG = "wifi_manager";

// Manager task configuration
#define MANAGER_TASK_STACK_SIZE     4096
#define MANAGER_TASK_PRIORITY       5
#define MANAGER_TASK_CORE           0

// Monitoring intervals (milliseconds)
#define SIGNAL_CHECK_INTERVAL_MS    5000
#define RECONNECT_CHECK_INTERVAL_MS 10000
#define ROAMING_CHECK_INTERVAL_MS   30000

// Signal thresholds
#define RSSI_ROAMING_THRESHOLD      -75     // dBm - consider roaming below this
#define RSSI_DISCONNECT_THRESHOLD   -85     // dBm - force reconnect below this
#define RSSI_HYSTERESIS             5       // dB - prevent ping-pong roaming

// Connection health
#define MAX_CONSECUTIVE_FAILURES    3
#define HEALTH_CHECK_TIMEOUT_MS     5000

// ============================================================================
// Type Definitions
// ============================================================================

/**
 * @brief Connection quality metrics
 */
typedef struct {
    int8_t rssi;                    ///< Current RSSI
    int8_t rssi_avg;                ///< Average RSSI (moving average)
    uint8_t channel;                ///< Current channel
    uint32_t tx_failures;           ///< TX failures count
    uint32_t rx_errors;             ///< RX errors count
    uint32_t connection_time_ms;    ///< Time connected in ms
    bool is_healthy;                ///< Connection health status
} connection_quality_t;

/**
 * @brief Manager state
 */
typedef enum {
    MANAGER_STATE_IDLE = 0,
    MANAGER_STATE_CONNECTING,
    MANAGER_STATE_CONNECTED,
    MANAGER_STATE_SCANNING,
    MANAGER_STATE_ROAMING
} manager_state_t;

// ============================================================================
// Static Variables
// ============================================================================

static bool s_manager_initialized = false;
static TaskHandle_t s_manager_task_handle = NULL;
static SemaphoreHandle_t s_manager_mutex = NULL;
static TimerHandle_t s_signal_timer = NULL;

static manager_state_t s_manager_state = MANAGER_STATE_IDLE;
static connection_quality_t s_conn_quality;
static int64_t s_connection_start_time = 0;

static bool s_roaming_enabled = false;
static bool s_auto_switch_enabled = true;
static int s_current_network_index = -1;
static uint32_t s_consecutive_failures = 0;

// ============================================================================
// Forward Declarations
// ============================================================================

static void manager_task(void *pvParameters);
static void signal_timer_callback(TimerHandle_t xTimer);
static void update_connection_quality(void);
static bool should_roam(void);
static int find_better_network(void);
static void attempt_roam(int target_network_index);

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize the WiFi manager
 *
 * Starts the manager task and timers for monitoring WiFi connection
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_init(void)
{
    if (s_manager_initialized) {
        ESP_LOGW(TAG, "Manager already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing WiFi manager...");

    // Create mutex
    s_manager_mutex = xSemaphoreCreateMutex();
    if (s_manager_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize quality metrics
    memset(&s_conn_quality, 0, sizeof(s_conn_quality));
    s_conn_quality.rssi = -127;
    s_conn_quality.rssi_avg = -127;

    // Create signal monitoring timer
    s_signal_timer = xTimerCreate("sig_timer",
                                   pdMS_TO_TICKS(SIGNAL_CHECK_INTERVAL_MS),
                                   pdTRUE,  // Auto-reload
                                   NULL,
                                   signal_timer_callback);

    if (s_signal_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create timer");
        vSemaphoreDelete(s_manager_mutex);
        return ESP_ERR_NO_MEM;
    }

    // Create manager task
    BaseType_t ret = xTaskCreatePinnedToCore(
        manager_task,
        "wifi_mgr",
        MANAGER_TASK_STACK_SIZE,
        NULL,
        MANAGER_TASK_PRIORITY,
        &s_manager_task_handle,
        MANAGER_TASK_CORE
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create manager task");
        xTimerDelete(s_signal_timer, 0);
        vSemaphoreDelete(s_manager_mutex);
        return ESP_ERR_NO_MEM;
    }

    // Start signal timer
    xTimerStart(s_signal_timer, 0);

    s_manager_initialized = true;
    ESP_LOGI(TAG, "WiFi manager initialized");

    return ESP_OK;
}

/**
 * @brief Deinitialize the WiFi manager
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_deinit(void)
{
    if (!s_manager_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing WiFi manager...");

    // Stop and delete timer
    if (s_signal_timer) {
        xTimerStop(s_signal_timer, 0);
        xTimerDelete(s_signal_timer, portMAX_DELAY);
        s_signal_timer = NULL;
    }

    // Delete task
    if (s_manager_task_handle) {
        vTaskDelete(s_manager_task_handle);
        s_manager_task_handle = NULL;
    }

    // Delete mutex
    if (s_manager_mutex) {
        vSemaphoreDelete(s_manager_mutex);
        s_manager_mutex = NULL;
    }

    s_manager_initialized = false;
    ESP_LOGI(TAG, "WiFi manager deinitialized");

    return ESP_OK;
}

// ============================================================================
// Configuration Functions
// ============================================================================

/**
 * @brief Enable/disable roaming
 *
 * When enabled, the manager will automatically switch to better networks
 * when signal quality degrades
 *
 * @param enable true to enable roaming
 */
void wifi_manager_enable_roaming(bool enable)
{
    s_roaming_enabled = enable;
    ESP_LOGI(TAG, "Roaming %s", enable ? "enabled" : "disabled");
}

/**
 * @brief Enable/disable auto network switching
 *
 * When enabled, automatically switches between stored networks
 * based on availability and signal strength
 *
 * @param enable true to enable auto switching
 */
void wifi_manager_enable_auto_switch(bool enable)
{
    s_auto_switch_enabled = enable;
    ESP_LOGI(TAG, "Auto-switch %s", enable ? "enabled" : "disabled");
}

/**
 * @brief Get current connection quality
 *
 * @param quality Pointer to quality structure to fill
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_get_quality(connection_quality_t *quality)
{
    if (!quality) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_manager_mutex, portMAX_DELAY);
    memcpy(quality, &s_conn_quality, sizeof(connection_quality_t));
    xSemaphoreGive(s_manager_mutex);

    return ESP_OK;
}

/**
 * @brief Check if connection is healthy
 *
 * @return true if connection is healthy
 */
bool wifi_manager_is_healthy(void)
{
    return s_conn_quality.is_healthy;
}

/**
 * @brief Get manager state
 *
 * @return Current manager state
 */
manager_state_t wifi_manager_get_state(void)
{
    return s_manager_state;
}

/**
 * @brief Force reconnection
 *
 * Disconnects from current network and reconnects (possibly to a different network)
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_force_reconnect(void)
{
    ESP_LOGI(TAG, "Forcing reconnection...");

    xSemaphoreTake(s_manager_mutex, portMAX_DELAY);
    s_consecutive_failures = 0;
    s_manager_state = MANAGER_STATE_CONNECTING;
    xSemaphoreGive(s_manager_mutex);

    // Disconnect and let event handler reconnect
    esp_wifi_disconnect();

    return ESP_OK;
}

/**
 * @brief Connect to next available network
 *
 * Tries to connect to the next network in the stored list
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_try_next_network(void)
{
    wifi_network_entry_t networks[WIFI_CONFIG_MAX_STORED_NETWORKS];
    int count = wifi_config_get_networks(networks, WIFI_CONFIG_MAX_STORED_NETWORKS);

    if (count == 0) {
        ESP_LOGW(TAG, "No networks configured");
        return ESP_ERR_NOT_FOUND;
    }

    // Find next valid network
    int start_index = (s_current_network_index + 1) % count;
    for (int i = 0; i < count; i++) {
        int index = (start_index + i) % count;
        if (networks[index].valid) {
            ESP_LOGI(TAG, "Trying network: %s", networks[index].ssid);
            s_current_network_index = index;
            return wifi_config_connect(networks[index].ssid, networks[index].password);
        }
    }

    return ESP_ERR_NOT_FOUND;
}

// ============================================================================
// Internal Functions
// ============================================================================

/**
 * @brief Signal monitoring timer callback
 */
static void signal_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (s_manager_state == MANAGER_STATE_CONNECTED) {
        update_connection_quality();
    }
}

/**
 * @brief Update connection quality metrics
 */
static void update_connection_quality(void)
{
    wifi_ap_record_t ap_info;

    if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
        return;
    }

    xSemaphoreTake(s_manager_mutex, portMAX_DELAY);

    // Update RSSI
    s_conn_quality.rssi = ap_info.rssi;
    s_conn_quality.channel = ap_info.primary;

    // Calculate moving average RSSI (simple exponential moving average)
    if (s_conn_quality.rssi_avg == -127) {
        s_conn_quality.rssi_avg = s_conn_quality.rssi;
    } else {
        s_conn_quality.rssi_avg = (s_conn_quality.rssi_avg * 7 + s_conn_quality.rssi) / 8;
    }

    // Update connection time
    if (s_connection_start_time > 0) {
        s_conn_quality.connection_time_ms = (esp_timer_get_time() - s_connection_start_time) / 1000;
    }

    // Check health
    s_conn_quality.is_healthy = (s_conn_quality.rssi > RSSI_DISCONNECT_THRESHOLD);

    xSemaphoreGive(s_manager_mutex);

    // Check if we should roam
    if (s_roaming_enabled && should_roam()) {
        int better_network = find_better_network();
        if (better_network >= 0) {
            attempt_roam(better_network);
        }
    }

    // Check for very poor signal
    if (s_conn_quality.rssi < RSSI_DISCONNECT_THRESHOLD) {
        ESP_LOGW(TAG, "Signal very weak (%d dBm), considering reconnection", s_conn_quality.rssi);
        s_consecutive_failures++;

        if (s_consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
            ESP_LOGW(TAG, "Too many failures, forcing reconnection");
            wifi_manager_force_reconnect();
        }
    } else {
        s_consecutive_failures = 0;
    }
}

/**
 * @brief Check if we should attempt to roam
 *
 * @return true if roaming conditions are met
 */
static bool should_roam(void)
{
    // Don't roam if signal is acceptable
    if (s_conn_quality.rssi_avg > RSSI_ROAMING_THRESHOLD) {
        return false;
    }

    // Don't roam if recently connected (avoid ping-pong)
    if (s_conn_quality.connection_time_ms < ROAMING_CHECK_INTERVAL_MS) {
        return false;
    }

    return true;
}

/**
 * @brief Find a better network to connect to
 *
 * @return Index of better network, or -1 if none found
 */
static int find_better_network(void)
{
    // Start a scan to find available networks
    if (wifi_config_is_scanning()) {
        return -1;
    }

    // Get scan results
    wifi_scan_result_t scan_results[20];
    int scan_count = wifi_config_get_scan_results(scan_results, 20);

    if (scan_count <= 0) {
        // Trigger a scan for next time
        wifi_config_start_scan();
        return -1;
    }

    // Get stored networks
    wifi_network_entry_t stored[WIFI_CONFIG_MAX_STORED_NETWORKS];
    int stored_count = wifi_config_get_networks(stored, WIFI_CONFIG_MAX_STORED_NETWORKS);

    int best_index = -1;
    int8_t best_rssi = s_conn_quality.rssi_avg + RSSI_HYSTERESIS;  // Must be significantly better

    // Find the best available stored network
    for (int i = 0; i < stored_count; i++) {
        if (!stored[i].valid) continue;

        // Check if this network is in scan results
        for (int j = 0; j < scan_count; j++) {
            if (strcmp(stored[i].ssid, scan_results[j].ssid) == 0) {
                if (scan_results[j].rssi > best_rssi) {
                    best_rssi = scan_results[j].rssi;
                    best_index = i;
                }
                break;
            }
        }
    }

    if (best_index >= 0 && best_index != s_current_network_index) {
        ESP_LOGI(TAG, "Found better network: %s (RSSI: %d vs current: %d)",
                 stored[best_index].ssid, best_rssi, s_conn_quality.rssi_avg);
    }

    return best_index;
}

/**
 * @brief Attempt to roam to a better network
 *
 * @param target_network_index Index of target network
 */
static void attempt_roam(int target_network_index)
{
    wifi_network_entry_t networks[WIFI_CONFIG_MAX_STORED_NETWORKS];
    int count = wifi_config_get_networks(networks, WIFI_CONFIG_MAX_STORED_NETWORKS);

    if (target_network_index < 0 || target_network_index >= count) {
        return;
    }

    ESP_LOGI(TAG, "Roaming to: %s", networks[target_network_index].ssid);

    xSemaphoreTake(s_manager_mutex, portMAX_DELAY);
    s_manager_state = MANAGER_STATE_ROAMING;
    s_current_network_index = target_network_index;
    xSemaphoreGive(s_manager_mutex);

    wifi_config_connect(networks[target_network_index].ssid,
                        networks[target_network_index].password);
}

/**
 * @brief Manager task - main loop for WiFi management
 */
static void manager_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Manager task started");

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        // Check connection state
        wifi_connection_state_t conn_state = wifi_config_get_connection_state();

        xSemaphoreTake(s_manager_mutex, portMAX_DELAY);

        switch (conn_state) {
            case WIFI_STATE_GOT_IP:
                if (s_manager_state != MANAGER_STATE_CONNECTED) {
                    s_manager_state = MANAGER_STATE_CONNECTED;
                    s_connection_start_time = esp_timer_get_time();
                    s_consecutive_failures = 0;
                    ESP_LOGI(TAG, "Connection established");
                }
                break;

            case WIFI_STATE_CONNECTING:
                s_manager_state = MANAGER_STATE_CONNECTING;
                break;

            case WIFI_STATE_DISCONNECTED:
            case WIFI_STATE_CONNECTION_FAILED:
                if (s_manager_state == MANAGER_STATE_CONNECTED ||
                    s_manager_state == MANAGER_STATE_ROAMING) {
                    s_manager_state = MANAGER_STATE_IDLE;
                    s_connection_start_time = 0;
                    memset(&s_conn_quality, 0, sizeof(s_conn_quality));
                    s_conn_quality.rssi = -127;
                    s_conn_quality.rssi_avg = -127;

                    // Try next network if auto-switch is enabled
                    if (s_auto_switch_enabled && conn_state == WIFI_STATE_CONNECTION_FAILED) {
                        xSemaphoreGive(s_manager_mutex);
                        wifi_manager_try_next_network();
                        xSemaphoreTake(s_manager_mutex, portMAX_DELAY);
                    }
                }
                break;

            default:
                break;
        }

        xSemaphoreGive(s_manager_mutex);

        // Wait for next iteration
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// Advanced Features
// ============================================================================

/**
 * @brief Get RSSI as percentage (0-100)
 *
 * @param rssi RSSI value in dBm
 * @return Signal quality percentage
 */
uint8_t wifi_manager_rssi_to_percent(int8_t rssi)
{
    if (rssi >= -50) return 100;
    if (rssi <= -100) return 0;

    // Linear interpolation between -50 and -100 dBm
    return (uint8_t)(2 * (rssi + 100));
}

/**
 * @brief Get signal quality description
 *
 * @param rssi RSSI value in dBm
 * @return Quality description string
 */
const char *wifi_manager_signal_quality_str(int8_t rssi)
{
    if (rssi >= -50) return "Excellent";
    if (rssi >= -60) return "Good";
    if (rssi >= -70) return "Fair";
    if (rssi >= -80) return "Weak";
    return "Poor";
}

/**
 * @brief Print connection diagnostics
 */
void wifi_manager_print_diagnostics(void)
{
    wifi_system_status_t status;
    wifi_config_get_status(&status);

    ESP_LOGI(TAG, "=== WiFi Diagnostics ===");
    ESP_LOGI(TAG, "Mode: %d", status.current_mode);
    ESP_LOGI(TAG, "State: %d", status.sta_state);
    ESP_LOGI(TAG, "SSID: %s", status.connected_ssid);
    ESP_LOGI(TAG, "RSSI: %d dBm (%s)",
             s_conn_quality.rssi,
             wifi_manager_signal_quality_str(s_conn_quality.rssi));
    ESP_LOGI(TAG, "RSSI Avg: %d dBm", s_conn_quality.rssi_avg);
    ESP_LOGI(TAG, "Channel: %d", s_conn_quality.channel);
    ESP_LOGI(TAG, "Uptime: %lu ms", (unsigned long)s_conn_quality.connection_time_ms);
    ESP_LOGI(TAG, "Healthy: %s", s_conn_quality.is_healthy ? "Yes" : "No");
    ESP_LOGI(TAG, "Manager State: %d", s_manager_state);
    ESP_LOGI(TAG, "Roaming: %s", s_roaming_enabled ? "Enabled" : "Disabled");
    ESP_LOGI(TAG, "========================");
}

/**
 * @brief Schedule a background scan
 *
 * Triggers a WiFi scan without interrupting current connection
 */
void wifi_manager_schedule_scan(void)
{
    if (!wifi_config_is_scanning()) {
        xSemaphoreTake(s_manager_mutex, portMAX_DELAY);
        s_manager_state = MANAGER_STATE_SCANNING;
        xSemaphoreGive(s_manager_mutex);

        wifi_config_start_scan();
    }
}

/**
 * @brief Get the currently connected network index
 *
 * @return Network index or -1 if not connected
 */
int wifi_manager_get_current_network_index(void)
{
    return s_current_network_index;
}

/**
 * @brief Set preferred network by index
 *
 * @param index Network index
 */
void wifi_manager_set_preferred_network(int index)
{
    if (index >= 0 && index < WIFI_CONFIG_MAX_STORED_NETWORKS) {
        s_current_network_index = index;
    }
}
