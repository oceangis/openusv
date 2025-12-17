/**
 * @file wifi_config.c
 * @brief WiFi Configuration System Main Implementation
 *
 * Handles WiFi initialization, AP/STA mode switching, HTTP server for
 * web configuration, and NVS storage of credentials.
 */

#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "wifi_config.h"

// ============================================================================
// Constants and Definitions
// ============================================================================

static const char *TAG = "wifi_config";

// NVS namespace and keys
#define NVS_NAMESPACE           "wifi_cfg"
#define NVS_KEY_CONFIG          "config"
#define NVS_KEY_NETWORKS        "networks"

// Event bits
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1
#define WIFI_SCAN_DONE_BIT      BIT2
#define WIFI_GOT_IP_BIT         BIT3

// HTTP Response buffers
#define HTTP_RESPONSE_SIZE      4096

// ============================================================================
// Static Variables
// ============================================================================

static bool s_initialized = false;
static wifi_system_config_t s_config;
static wifi_system_status_t s_status;
static wifi_mavlink_stats_t s_mavlink_stats;

static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;
static httpd_handle_t s_http_server = NULL;

static EventGroupHandle_t s_wifi_event_group = NULL;
static SemaphoreHandle_t s_config_mutex = NULL;

static wifi_event_callback_t s_event_callback = NULL;
static void *s_callback_user_data = NULL;

// External storage callback (for AP_Param Flash storage)
static wifi_config_save_callback_t s_save_callback = NULL;
static bool s_use_external_storage = false;

// MAVLink UDP socket
static int s_mavlink_socket = -1;
static struct sockaddr_in s_mavlink_target_addr;

// Scan results
static wifi_scan_result_t *s_scan_results = NULL;
static int s_scan_result_count = 0;
static bool s_scan_in_progress = false;

// Reconnection management
static int s_reconnect_attempts = 0;
#define MAX_RECONNECT_ATTEMPTS  5

// ============================================================================
// Forward Declarations
// ============================================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data);
static esp_err_t start_http_server(void);
static void stop_http_server(void);
static esp_err_t load_config_from_nvs(void);
static esp_err_t save_config_to_nvs(void);
static void set_default_config(void);
static void notify_state_change(wifi_connection_state_t state);

// HTTP handlers
static esp_err_t http_get_handler(httpd_req_t *req);
static esp_err_t http_api_status_handler(httpd_req_t *req);
static esp_err_t http_api_scan_handler(httpd_req_t *req);
static esp_err_t http_api_connect_handler(httpd_req_t *req);
static esp_err_t http_api_config_handler(httpd_req_t *req);

// ============================================================================
// HTML Content for Web Configuration Interface
// ============================================================================

static const char HTML_PAGE[] =
"<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>USV WiFi Config</title><style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;padding:20px}"
".container{max-width:600px;margin:0 auto}"
"h1{color:#00d4ff;text-align:center;margin-bottom:20px}"
".card{background:#16213e;border-radius:10px;padding:20px;margin-bottom:15px}"
".card h2{color:#00d4ff;margin-bottom:15px;font-size:1.2em}"
".status{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #0f3460}"
".status:last-child{border:none}"
".status-label{color:#aaa}"
".status-value{color:#00d4ff}"
".connected{color:#00ff88}"
".disconnected{color:#ff4444}"
".btn{background:#00d4ff;color:#16213e;border:none;padding:12px 24px;border-radius:5px;cursor:pointer;font-weight:bold;width:100%;margin-top:10px}"
".btn:hover{background:#00a8cc}"
".btn-danger{background:#ff4444}"
".btn-danger:hover{background:#cc3333}"
"input,select{width:100%;padding:12px;margin:8px 0;border:none;border-radius:5px;background:#0f3460;color:#eee}"
"input:focus,select:focus{outline:2px solid #00d4ff}"
".network-list{max-height:300px;overflow-y:auto}"
".network-item{display:flex;justify-content:space-between;align-items:center;padding:12px;background:#0f3460;border-radius:5px;margin-bottom:8px;cursor:pointer}"
".network-item:hover{background:#1a3a5c}"
".rssi{font-size:0.9em}"
".rssi-good{color:#00ff88}"
".rssi-medium{color:#ffaa00}"
".rssi-weak{color:#ff4444}"
".loader{border:3px solid #0f3460;border-top:3px solid #00d4ff;border-radius:50%;width:20px;height:20px;animation:spin 1s linear infinite;display:inline-block;margin-left:10px}"
"@keyframes spin{0%{transform:rotate(0deg)}100%{transform:rotate(360deg)}}"
".hidden{display:none}"
"</style></head><body>"
"<div class='container'>"
"<h1>USV WiFi Configuration</h1>"
"<div class='card'><h2>System Status</h2>"
"<div class='status'><span class='status-label'>Mode</span><span id='mode' class='status-value'>-</span></div>"
"<div class='status'><span class='status-label'>Connection</span><span id='conn-status' class='status-value'>-</span></div>"
"<div class='status'><span class='status-label'>SSID</span><span id='ssid' class='status-value'>-</span></div>"
"<div class='status'><span class='status-label'>IP Address</span><span id='ip' class='status-value'>-</span></div>"
"<div class='status'><span class='status-label'>Signal</span><span id='rssi' class='status-value'>-</span></div>"
"</div>"
"<div class='card'><h2>Available Networks <span id='scan-loader' class='loader hidden'></span></h2>"
"<div id='network-list' class='network-list'><p>Click 'Scan' to find networks</p></div>"
"<button class='btn' onclick='scanNetworks()'>Scan Networks</button></div>"
"<div class='card'><h2>Connect to Network</h2>"
"<input type='text' id='wifi-ssid' placeholder='WiFi SSID'>"
"<input type='password' id='wifi-pass' placeholder='WiFi Password'>"
"<button class='btn' onclick='connectWifi()'>Connect</button></div>"
"<div class='card'><h2>AP Settings</h2>"
"<input type='text' id='ap-ssid' placeholder='AP SSID'>"
"<input type='password' id='ap-pass' placeholder='AP Password (8+ chars or empty)'>"
"<button class='btn' onclick='saveApConfig()'>Save AP Settings</button></div>"
"<div class='card'><h2>Actions</h2>"
"<button class='btn' onclick='disconnect()'>Disconnect</button>"
"<button class='btn btn-danger' onclick='resetConfig()'>Reset to Defaults</button></div></div>"
"<script>"
"function updateStatus(){fetch('/api/status').then(r=>r.json()).then(d=>{"
"document.getElementById('mode').textContent=d.mode;"
"let cs=document.getElementById('conn-status');"
"cs.textContent=d.state;cs.className='status-value '+(d.state=='GOT_IP'?'connected':'disconnected');"
"document.getElementById('ssid').textContent=d.ssid||'-';"
"document.getElementById('ip').textContent=d.ip||'-';"
"document.getElementById('rssi').textContent=d.rssi?d.rssi+' dBm':'-';"
"}).catch(e=>console.error(e))}"
"function scanNetworks(){document.getElementById('scan-loader').classList.remove('hidden');"
"fetch('/api/scan',{method:'POST'}).then(r=>r.json()).then(d=>{setTimeout(getScanResults,3000)}).catch(e=>{document.getElementById('scan-loader').classList.add('hidden');alert('Scan failed')})}"
"function getScanResults(){fetch('/api/scan').then(r=>r.json()).then(d=>{"
"document.getElementById('scan-loader').classList.add('hidden');"
"let list=document.getElementById('network-list');"
"if(d.networks&&d.networks.length>0){"
"list.innerHTML=d.networks.map(n=>"
"'<div class=\"network-item\" onclick=\"selectNetwork(\\''+n.ssid+'\\')\">'+"
"'<span>'+n.ssid+'</span>'+"
"'<span class=\"rssi '+(n.rssi>-50?'rssi-good':n.rssi>-70?'rssi-medium':'rssi-weak')+'\">'+n.rssi+' dBm</span></div>').join('')"
"}else{list.innerHTML='<p>No networks found</p>'}}).catch(e=>{document.getElementById('scan-loader').classList.add('hidden')})}"
"function selectNetwork(ssid){document.getElementById('wifi-ssid').value=ssid;document.getElementById('wifi-pass').focus()}"
"function connectWifi(){let ssid=document.getElementById('wifi-ssid').value,pass=document.getElementById('wifi-pass').value;"
"if(!ssid){alert('Enter SSID');return}"
"fetch('/api/connect',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid:ssid,password:pass})})"
".then(r=>r.json()).then(d=>{alert(d.message||'Connecting...');setTimeout(updateStatus,5000)}).catch(e=>alert('Error'))}"
"function disconnect(){fetch('/api/connect',{method:'DELETE'}).then(r=>r.json()).then(d=>{alert(d.message);updateStatus()}).catch(e=>alert('Error'))}"
"function saveApConfig(){let ssid=document.getElementById('ap-ssid').value,pass=document.getElementById('ap-pass').value;"
"fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ap_ssid:ssid,ap_password:pass})})"
".then(r=>r.json()).then(d=>alert(d.message||'Saved')).catch(e=>alert('Error'))}"
"function resetConfig(){if(confirm('Reset all settings?')){"
"fetch('/api/config',{method:'DELETE'}).then(r=>r.json()).then(d=>{alert(d.message);location.reload()}).catch(e=>alert('Error'))}}"
"updateStatus();setInterval(updateStatus,5000);"
"</script></body></html>";

// ============================================================================
// System Initialization
// ============================================================================

esp_err_t wifi_config_system_init(void)
{
    esp_err_t ret;

    if (s_initialized) {
        ESP_LOGW(TAG, "WiFi config already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing WiFi configuration system...");

    // Create synchronization primitives
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    s_config_mutex = xSemaphoreCreateMutex();
    if (s_config_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        vEventGroupDelete(s_wifi_event_group);
        return ESP_ERR_NO_MEM;
    }

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Load or create default configuration
    ret = load_config_from_nvs();
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "No saved config, using defaults");
        set_default_config();
        save_config_to_nvs();
    }

    // Initialize TCP/IP stack
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Netif init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create default event loop
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Event loop create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create network interfaces
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register event handlers
    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler");
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                              &ip_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler");
        return ret;
    }

    // Set storage mode
    ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi storage");
        return ret;
    }

    // Initialize status
    memset(&s_status, 0, sizeof(s_status));
    memset(&s_mavlink_stats, 0, sizeof(s_mavlink_stats));

    // Check if we have any stored networks
    bool has_sta_config = false;
    for (int i = 0; i < WIFI_CONFIG_MAX_STORED_NETWORKS; i++) {
        if (s_config.sta_networks[i].valid) {
            has_sta_config = true;
            break;
        }
    }

    // Start in appropriate mode
    if (has_sta_config) {
        ESP_LOGI(TAG, "Found stored networks, starting in APSTA mode");
        ret = wifi_config_set_mode(WIFI_CONFIG_MODE_APSTA);
    } else {
        ESP_LOGI(TAG, "No stored networks, starting in AP mode for configuration");
        ret = wifi_config_set_mode(WIFI_CONFIG_MODE_AP);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode");
        return ret;
    }

    // Start HTTP server
    ret = start_http_server();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "HTTP server start failed, web config unavailable");
    }

    // Initialize MAVLink UDP if enabled
    if (s_config.mavlink_udp_enabled) {
        wifi_config_mavlink_enable(true, s_config.mavlink_udp_target_ip, s_config.mavlink_udp_port);
    }

    s_initialized = true;
    ESP_LOGI(TAG, "WiFi configuration system initialized");

    return ESP_OK;
}

esp_err_t wifi_config_system_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing WiFi configuration system...");

    // Close MAVLink socket
    if (s_mavlink_socket >= 0) {
        close(s_mavlink_socket);
        s_mavlink_socket = -1;
    }

    // Stop HTTP server
    stop_http_server();

    // Stop WiFi
    esp_wifi_stop();
    esp_wifi_deinit();

    // Free scan results
    if (s_scan_results) {
        free(s_scan_results);
        s_scan_results = NULL;
    }

    // Delete synchronization primitives
    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    if (s_config_mutex) {
        vSemaphoreDelete(s_config_mutex);
        s_config_mutex = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "WiFi configuration system deinitialized");

    return ESP_OK;
}

bool wifi_config_is_initialized(void)
{
    return s_initialized;
}

// ============================================================================
// Event Handlers
// ============================================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "STA started, connecting...");
            s_status.sta_state = WIFI_STATE_CONNECTING;
            // Try to connect to first available network
            wifi_config_start_sta();
            break;

        case WIFI_EVENT_STA_CONNECTED: {
            wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
            ESP_LOGI(TAG, "Connected to AP: %s, channel: %d", event->ssid, event->channel);
            s_status.sta_state = WIFI_STATE_CONNECTED;
            s_status.current_channel = event->channel;
            strncpy(s_status.connected_ssid, (char *)event->ssid, sizeof(s_status.connected_ssid) - 1);
            s_reconnect_attempts = 0;
            notify_state_change(WIFI_STATE_CONNECTED);
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGW(TAG, "Disconnected from AP, reason: %d", event->reason);
            s_status.sta_state = WIFI_STATE_DISCONNECTED;
            s_status.connected_ssid[0] = '\0';
            s_status.current_rssi = 0;
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);

            // Auto reconnect logic
            if (s_config.auto_reconnect && s_reconnect_attempts < MAX_RECONNECT_ATTEMPTS) {
                s_reconnect_attempts++;
                s_status.reconnect_count++;
                ESP_LOGI(TAG, "Reconnect attempt %d/%d", s_reconnect_attempts, MAX_RECONNECT_ATTEMPTS);
                vTaskDelay(pdMS_TO_TICKS(WIFI_CONFIG_RECONNECT_DELAY_MS));
                esp_wifi_connect();
            } else if (s_reconnect_attempts >= MAX_RECONNECT_ATTEMPTS) {
                ESP_LOGW(TAG, "Max reconnect attempts reached");
                s_status.sta_state = WIFI_STATE_CONNECTION_FAILED;
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            notify_state_change(WIFI_STATE_DISCONNECTED);
            break;
        }

        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "AP started");
            s_status.ap_active = true;
            break;

        case WIFI_EVENT_AP_STOP:
            ESP_LOGI(TAG, "AP stopped");
            s_status.ap_active = false;
            break;

        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
            ESP_LOGI(TAG, "Station " MACSTR " connected to AP, AID=%d",
                     MAC2STR(event->mac), event->aid);
            s_status.connected_clients++;
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
            ESP_LOGI(TAG, "Station " MACSTR " disconnected from AP, AID=%d",
                     MAC2STR(event->mac), event->aid);
            if (s_status.connected_clients > 0) {
                s_status.connected_clients--;
            }
            break;
        }

        case WIFI_EVENT_SCAN_DONE: {
            ESP_LOGI(TAG, "WiFi scan done");
            s_scan_in_progress = false;

            uint16_t ap_count = 0;
            esp_wifi_scan_get_ap_num(&ap_count);

            if (ap_count > 0) {
                wifi_ap_record_t *ap_records = malloc(sizeof(wifi_ap_record_t) * ap_count);
                if (ap_records) {
                    esp_wifi_scan_get_ap_records(&ap_count, ap_records);

                    // Free old results
                    if (s_scan_results) {
                        free(s_scan_results);
                    }

                    s_scan_results = malloc(sizeof(wifi_scan_result_t) * ap_count);
                    if (s_scan_results) {
                        s_scan_result_count = ap_count;
                        for (int i = 0; i < ap_count; i++) {
                            strncpy(s_scan_results[i].ssid, (char *)ap_records[i].ssid,
                                    sizeof(s_scan_results[i].ssid) - 1);
                            s_scan_results[i].rssi = ap_records[i].rssi;
                            s_scan_results[i].channel = ap_records[i].primary;
                            s_scan_results[i].authmode = ap_records[i].authmode;
                            memcpy(s_scan_results[i].bssid, ap_records[i].bssid, 6);
                        }
                    }
                    free(ap_records);
                }
            }
            xEventGroupSetBits(s_wifi_event_group, WIFI_SCAN_DONE_BIT);
            break;
        }

        default:
            break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    switch (event_id) {
        case IP_EVENT_STA_GOT_IP: {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            s_status.sta_state = WIFI_STATE_GOT_IP;
            memcpy(&s_status.sta_ip_info, &event->ip_info, sizeof(esp_netif_ip_info_t));
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);
            notify_state_change(WIFI_STATE_GOT_IP);
            break;
        }

        case IP_EVENT_STA_LOST_IP:
            ESP_LOGW(TAG, "Lost IP address");
            s_status.sta_state = WIFI_STATE_LOST_IP;
            memset(&s_status.sta_ip_info, 0, sizeof(esp_netif_ip_info_t));
            xEventGroupClearBits(s_wifi_event_group, WIFI_GOT_IP_BIT);
            notify_state_change(WIFI_STATE_LOST_IP);
            break;

        case IP_EVENT_AP_STAIPASSIGNED: {
            ip_event_ap_staipassigned_t *event = (ip_event_ap_staipassigned_t *)event_data;
            ESP_LOGI(TAG, "AP assigned IP to station: " IPSTR, IP2STR(&event->ip));
            break;
        }

        default:
            break;
    }
}

// ============================================================================
// Mode Control Functions
// ============================================================================

esp_err_t wifi_config_set_mode(wifi_config_mode_t mode)
{
    esp_err_t ret;
    wifi_mode_t wifi_mode;

    ESP_LOGI(TAG, "Setting WiFi mode: %d", mode);

    switch (mode) {
        case WIFI_CONFIG_MODE_OFF:
            wifi_mode = WIFI_MODE_NULL;
            break;
        case WIFI_CONFIG_MODE_AP:
            wifi_mode = WIFI_MODE_AP;
            break;
        case WIFI_CONFIG_MODE_STA:
            wifi_mode = WIFI_MODE_STA;
            break;
        case WIFI_CONFIG_MODE_APSTA:
            wifi_mode = WIFI_MODE_APSTA;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    ret = esp_wifi_set_mode(wifi_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    s_status.current_mode = mode;

    // Configure and start based on mode
    if (mode == WIFI_CONFIG_MODE_AP || mode == WIFI_CONFIG_MODE_APSTA) {
        ret = wifi_config_start_ap();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start AP");
        }
    }

    if (mode == WIFI_CONFIG_MODE_STA || mode == WIFI_CONFIG_MODE_APSTA) {
        // STA configuration will be done when wifi_config_start_sta() is called
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

wifi_config_mode_t wifi_config_get_mode(void)
{
    return s_status.current_mode;
}

esp_err_t wifi_config_start_ap(void)
{
    esp_err_t ret;
    wifi_config_t wifi_ap_config = {0};

    // Generate default SSID if not set
    if (strlen(s_config.ap_ssid) == 0) {
        wifi_config_get_default_ap_ssid(s_config.ap_ssid);
    }

    strncpy((char *)wifi_ap_config.ap.ssid, s_config.ap_ssid, sizeof(wifi_ap_config.ap.ssid) - 1);
    wifi_ap_config.ap.ssid_len = strlen(s_config.ap_ssid);
    wifi_ap_config.ap.channel = s_config.ap_channel ? s_config.ap_channel : WIFI_CONFIG_AP_DEFAULT_CHANNEL;
    wifi_ap_config.ap.max_connection = WIFI_CONFIG_AP_MAX_CONNECTIONS;
    wifi_ap_config.ap.ssid_hidden = s_config.ap_hidden ? 1 : 0;

    if (strlen(s_config.ap_password) >= 8) {
        strncpy((char *)wifi_ap_config.ap.password, s_config.ap_password, sizeof(wifi_ap_config.ap.password) - 1);
        wifi_ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set AP config: %s", esp_err_to_name(ret));
        return ret;
    }

    // Get AP IP info
    esp_netif_get_ip_info(s_ap_netif, &s_status.ap_ip_info);

    ESP_LOGI(TAG, "AP started: SSID=%s, Channel=%d, Auth=%d",
             s_config.ap_ssid, wifi_ap_config.ap.channel, wifi_ap_config.ap.authmode);

    return ESP_OK;
}

esp_err_t wifi_config_stop_ap(void)
{
    if (s_status.current_mode == WIFI_CONFIG_MODE_AP) {
        return wifi_config_set_mode(WIFI_CONFIG_MODE_OFF);
    } else if (s_status.current_mode == WIFI_CONFIG_MODE_APSTA) {
        return wifi_config_set_mode(WIFI_CONFIG_MODE_STA);
    }
    return ESP_OK;
}

esp_err_t wifi_config_start_sta(void)
{
    esp_err_t ret;

    // Find first valid network
    for (int i = 0; i < WIFI_CONFIG_MAX_STORED_NETWORKS; i++) {
        if (s_config.sta_networks[i].valid) {
            wifi_config_t wifi_sta_config = {0};
            strncpy((char *)wifi_sta_config.sta.ssid, s_config.sta_networks[i].ssid,
                    sizeof(wifi_sta_config.sta.ssid) - 1);
            strncpy((char *)wifi_sta_config.sta.password, s_config.sta_networks[i].password,
                    sizeof(wifi_sta_config.sta.password) - 1);

            ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set STA config: %s", esp_err_to_name(ret));
                continue;
            }

            ESP_LOGI(TAG, "Connecting to: %s", s_config.sta_networks[i].ssid);
            ret = esp_wifi_connect();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to connect: %s", esp_err_to_name(ret));
                continue;
            }

            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "No valid networks configured");
    return ESP_ERR_WIFI_NOT_CONNECT;
}

esp_err_t wifi_config_stop_sta(void)
{
    esp_wifi_disconnect();
    if (s_status.current_mode == WIFI_CONFIG_MODE_STA) {
        return wifi_config_set_mode(WIFI_CONFIG_MODE_OFF);
    } else if (s_status.current_mode == WIFI_CONFIG_MODE_APSTA) {
        return wifi_config_set_mode(WIFI_CONFIG_MODE_AP);
    }
    return ESP_OK;
}

// ============================================================================
// Configuration Management
// ============================================================================

static void set_default_config(void)
{
    memset(&s_config, 0, sizeof(s_config));

    s_config.default_mode = WIFI_CONFIG_MODE_AP;
    wifi_config_get_default_ap_ssid(s_config.ap_ssid);
    s_config.ap_channel = WIFI_CONFIG_AP_DEFAULT_CHANNEL;
    s_config.ap_hidden = false;
    s_config.auto_reconnect = true;
    s_config.mavlink_udp_enabled = false;
    s_config.mavlink_udp_port = WIFI_CONFIG_MAVLINK_UDP_PORT;
    s_config.connect_timeout_ms = WIFI_CONFIG_CONNECT_TIMEOUT_MS;

    ESP_LOGI(TAG, "Default config set, AP SSID: %s", s_config.ap_ssid);
}

static esp_err_t load_config_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    size_t size = sizeof(s_config);
    ret = nvs_get_blob(handle, NVS_KEY_CONFIG, &s_config, &size);
    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Config loaded from NVS");
    }

    return ret;
}

static esp_err_t save_config_to_nvs(void)
{
    // If external storage callback is registered, use it instead of NVS
    if (s_use_external_storage && s_save_callback) {
        esp_err_t ret = s_save_callback(&s_config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Config saved via external callback (AP_Param Flash)");
        } else {
            ESP_LOGE(TAG, "External save callback failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    // Fallback to NVS storage
    nvs_handle_t handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_blob(handle, NVS_KEY_CONFIG, &s_config, sizeof(s_config));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Config saved to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to save config: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t wifi_config_get_config(wifi_system_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    memcpy(config, &s_config, sizeof(wifi_system_config_t));
    xSemaphoreGive(s_config_mutex);

    return ESP_OK;
}

esp_err_t wifi_config_set_config(const wifi_system_config_t *config, bool save_to_nvs)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    memcpy(&s_config, config, sizeof(wifi_system_config_t));
    xSemaphoreGive(s_config_mutex);

    if (save_to_nvs) {
        return save_config_to_nvs();
    }

    return ESP_OK;
}

esp_err_t wifi_config_reset_defaults(void)
{
    xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    set_default_config();
    esp_err_t ret = save_config_to_nvs();
    xSemaphoreGive(s_config_mutex);

    return ret;
}

esp_err_t wifi_config_set_ap_config(const char *ssid, const char *password, uint8_t channel)
{
    xSemaphoreTake(s_config_mutex, portMAX_DELAY);

    if (ssid && strlen(ssid) > 0) {
        strncpy(s_config.ap_ssid, ssid, sizeof(s_config.ap_ssid) - 1);
    } else {
        wifi_config_get_default_ap_ssid(s_config.ap_ssid);
    }

    if (password) {
        strncpy(s_config.ap_password, password, sizeof(s_config.ap_password) - 1);
    } else {
        s_config.ap_password[0] = '\0';
    }

    if (channel > 0 && channel <= 13) {
        s_config.ap_channel = channel;
    }

    esp_err_t ret = save_config_to_nvs();
    xSemaphoreGive(s_config_mutex);

    return ret;
}

// ============================================================================
// Network Management
// ============================================================================

esp_err_t wifi_config_add_network(const char *ssid, const char *password, uint8_t priority)
{
    if (!ssid || strlen(ssid) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_config_mutex, portMAX_DELAY);

    // Find empty slot or existing entry
    int slot = -1;
    for (int i = 0; i < WIFI_CONFIG_MAX_STORED_NETWORKS; i++) {
        if (!s_config.sta_networks[i].valid) {
            if (slot < 0) slot = i;
        } else if (strcmp(s_config.sta_networks[i].ssid, ssid) == 0) {
            slot = i;  // Update existing
            break;
        }
    }

    if (slot < 0) {
        xSemaphoreGive(s_config_mutex);
        ESP_LOGW(TAG, "No space for new network");
        return ESP_ERR_NO_MEM;
    }

    strncpy(s_config.sta_networks[slot].ssid, ssid, sizeof(s_config.sta_networks[slot].ssid) - 1);
    if (password) {
        strncpy(s_config.sta_networks[slot].password, password,
                sizeof(s_config.sta_networks[slot].password) - 1);
    } else {
        s_config.sta_networks[slot].password[0] = '\0';
    }
    s_config.sta_networks[slot].priority = priority;
    s_config.sta_networks[slot].rssi = -127;
    s_config.sta_networks[slot].valid = true;

    esp_err_t ret = save_config_to_nvs();
    xSemaphoreGive(s_config_mutex);

    ESP_LOGI(TAG, "Network added: %s (slot %d)", ssid, slot);
    return ret;
}

esp_err_t wifi_config_remove_network(const char *ssid)
{
    if (!ssid) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_config_mutex, portMAX_DELAY);

    for (int i = 0; i < WIFI_CONFIG_MAX_STORED_NETWORKS; i++) {
        if (s_config.sta_networks[i].valid &&
            strcmp(s_config.sta_networks[i].ssid, ssid) == 0) {
            memset(&s_config.sta_networks[i], 0, sizeof(wifi_network_entry_t));
            esp_err_t ret = save_config_to_nvs();
            xSemaphoreGive(s_config_mutex);
            ESP_LOGI(TAG, "Network removed: %s", ssid);
            return ret;
        }
    }

    xSemaphoreGive(s_config_mutex);
    return ESP_ERR_NOT_FOUND;
}

int wifi_config_get_networks(wifi_network_entry_t *networks, int max_count)
{
    if (!networks || max_count <= 0) return 0;

    xSemaphoreTake(s_config_mutex, portMAX_DELAY);

    int count = 0;
    for (int i = 0; i < WIFI_CONFIG_MAX_STORED_NETWORKS && count < max_count; i++) {
        if (s_config.sta_networks[i].valid) {
            memcpy(&networks[count], &s_config.sta_networks[i], sizeof(wifi_network_entry_t));
            count++;
        }
    }

    xSemaphoreGive(s_config_mutex);
    return count;
}

esp_err_t wifi_config_clear_networks(void)
{
    xSemaphoreTake(s_config_mutex, portMAX_DELAY);

    for (int i = 0; i < WIFI_CONFIG_MAX_STORED_NETWORKS; i++) {
        memset(&s_config.sta_networks[i], 0, sizeof(wifi_network_entry_t));
    }

    esp_err_t ret = save_config_to_nvs();
    xSemaphoreGive(s_config_mutex);

    ESP_LOGI(TAG, "All networks cleared");
    return ret;
}

// ============================================================================
// Scanning
// ============================================================================

esp_err_t wifi_config_start_scan(void)
{
    if (s_scan_in_progress) {
        return ESP_ERR_INVALID_STATE;
    }

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300
    };

    s_scan_in_progress = true;
    xEventGroupClearBits(s_wifi_event_group, WIFI_SCAN_DONE_BIT);

    esp_err_t ret = esp_wifi_scan_start(&scan_config, false);
    if (ret != ESP_OK) {
        s_scan_in_progress = false;
        ESP_LOGE(TAG, "Scan start failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "WiFi scan started");
    }

    return ret;
}

int wifi_config_get_scan_results(wifi_scan_result_t *results, int max_count)
{
    if (s_scan_in_progress) {
        return -1;
    }

    if (!results || max_count <= 0 || !s_scan_results) {
        return 0;
    }

    int count = (s_scan_result_count < max_count) ? s_scan_result_count : max_count;
    memcpy(results, s_scan_results, sizeof(wifi_scan_result_t) * count);

    return count;
}

bool wifi_config_is_scanning(void)
{
    return s_scan_in_progress;
}

// ============================================================================
// Connection Management
// ============================================================================

esp_err_t wifi_config_connect(const char *ssid, const char *password)
{
    if (!ssid || strlen(ssid) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Connecting to: %s", ssid);

    wifi_config_t wifi_sta_config = {0};
    strncpy((char *)wifi_sta_config.sta.ssid, ssid, sizeof(wifi_sta_config.sta.ssid) - 1);
    if (password) {
        strncpy((char *)wifi_sta_config.sta.password, password, sizeof(wifi_sta_config.sta.password) - 1);
    }

    // Disconnect first if connected
    esp_wifi_disconnect();

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STA config: %s", esp_err_to_name(ret));
        return ret;
    }

    s_status.sta_state = WIFI_STATE_CONNECTING;
    s_reconnect_attempts = 0;

    ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect: %s", esp_err_to_name(ret));
        s_status.sta_state = WIFI_STATE_CONNECTION_FAILED;
    }

    return ret;
}

esp_err_t wifi_config_disconnect(void)
{
    s_config.auto_reconnect = false;  // Disable auto reconnect
    return esp_wifi_disconnect();
}

wifi_connection_state_t wifi_config_get_connection_state(void)
{
    return s_status.sta_state;
}

esp_err_t wifi_config_get_status(wifi_system_status_t *status)
{
    if (!status) return ESP_ERR_INVALID_ARG;

    // Update RSSI if connected
    if (s_status.sta_state == WIFI_STATE_GOT_IP) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_status.current_rssi = ap_info.rssi;
        }
    }

    memcpy(status, &s_status, sizeof(wifi_system_status_t));
    return ESP_OK;
}

esp_err_t wifi_config_register_callback(wifi_event_callback_t callback, void *user_data)
{
    s_event_callback = callback;
    s_callback_user_data = user_data;
    return ESP_OK;
}

esp_err_t wifi_config_register_save_callback(wifi_config_save_callback_t callback)
{
    s_save_callback = callback;
    s_use_external_storage = (callback != NULL);
    ESP_LOGI(TAG, "External storage callback %s", s_use_external_storage ? "registered" : "cleared");
    return ESP_OK;
}

bool wifi_config_use_external_storage(void)
{
    return s_use_external_storage;
}

static void notify_state_change(wifi_connection_state_t state)
{
    if (s_event_callback) {
        s_event_callback(state, s_callback_user_data);
    }
}

// ============================================================================
// MAVLink UDP Functions
// ============================================================================

esp_err_t wifi_config_mavlink_enable(bool enable, const char *target_ip, uint16_t port)
{
    if (!enable) {
        if (s_mavlink_socket >= 0) {
            close(s_mavlink_socket);
            s_mavlink_socket = -1;
        }
        s_config.mavlink_udp_enabled = false;
        ESP_LOGI(TAG, "MAVLink UDP disabled");
        return ESP_OK;
    }

    // Create UDP socket
    s_mavlink_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_mavlink_socket < 0) {
        ESP_LOGE(TAG, "Failed to create MAVLink socket");
        return ESP_FAIL;
    }

    // Set socket options
    int broadcast = 1;
    setsockopt(s_mavlink_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    // Set non-blocking
    int flags = fcntl(s_mavlink_socket, F_GETFL, 0);
    fcntl(s_mavlink_socket, F_SETFL, flags | O_NONBLOCK);

    // Configure target address
    memset(&s_mavlink_target_addr, 0, sizeof(s_mavlink_target_addr));
    s_mavlink_target_addr.sin_family = AF_INET;
    s_mavlink_target_addr.sin_port = htons(port ? port : WIFI_CONFIG_MAVLINK_UDP_PORT);

    if (target_ip && strlen(target_ip) > 0) {
        inet_pton(AF_INET, target_ip, &s_mavlink_target_addr.sin_addr);
        strncpy(s_config.mavlink_udp_target_ip, target_ip, sizeof(s_config.mavlink_udp_target_ip) - 1);
    } else {
        s_mavlink_target_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    }

    // Bind to receive
    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port ? port : WIFI_CONFIG_MAVLINK_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    if (bind(s_mavlink_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGW(TAG, "MAVLink socket bind failed (may already be bound)");
    }

    s_config.mavlink_udp_enabled = true;
    s_config.mavlink_udp_port = port ? port : WIFI_CONFIG_MAVLINK_UDP_PORT;

    ESP_LOGI(TAG, "MAVLink UDP enabled on port %d", s_config.mavlink_udp_port);
    return ESP_OK;
}

int wifi_config_mavlink_send(const uint8_t *data, size_t len)
{
    if (s_mavlink_socket < 0 || !data || len == 0) {
        return -1;
    }

    int sent = sendto(s_mavlink_socket, data, len, 0,
                      (struct sockaddr *)&s_mavlink_target_addr, sizeof(s_mavlink_target_addr));

    if (sent > 0) {
        s_mavlink_stats.tx_packets++;
        s_mavlink_stats.tx_bytes += sent;
    } else {
        s_mavlink_stats.tx_errors++;
    }

    return sent;
}

int wifi_config_mavlink_receive(uint8_t *data, size_t max_len)
{
    if (s_mavlink_socket < 0 || !data || max_len == 0) {
        return -1;
    }

    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);

    int received = recvfrom(s_mavlink_socket, data, max_len, 0,
                            (struct sockaddr *)&src_addr, &src_len);

    if (received > 0) {
        s_mavlink_stats.rx_packets++;
        s_mavlink_stats.rx_bytes += received;
        s_mavlink_stats.connected = true;
    } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        s_mavlink_stats.rx_errors++;
    }

    return received;
}

int wifi_config_mavlink_available(void)
{
    if (s_mavlink_socket < 0) {
        return 0;
    }

    int bytes_available = 0;
    ioctl(s_mavlink_socket, FIONREAD, &bytes_available);
    return bytes_available;
}

esp_err_t wifi_config_mavlink_get_stats(wifi_mavlink_stats_t *stats)
{
    if (!stats) return ESP_ERR_INVALID_ARG;
    memcpy(stats, &s_mavlink_stats, sizeof(wifi_mavlink_stats_t));
    return ESP_OK;
}

// ============================================================================
// Utility Functions
// ============================================================================

esp_err_t wifi_config_get_mac(uint8_t *mac)
{
    if (!mac) return ESP_ERR_INVALID_ARG;
    return esp_read_mac(mac, ESP_MAC_WIFI_STA);
}

esp_err_t wifi_config_get_default_ap_ssid(char *ssid)
{
    if (!ssid) return ESP_ERR_INVALID_ARG;

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    sprintf(ssid, "USV_%02X%02X%02X", mac[3], mac[4], mac[5]);

    return ESP_OK;
}

const char *wifi_config_authmode_str(uint8_t authmode)
{
    switch (authmode) {
        case WIFI_AUTH_OPEN: return "OPEN";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA";
        case WIFI_AUTH_WPA2_PSK: return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
        case WIFI_AUTH_WPA3_PSK: return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// HTTP Server
// ============================================================================

static esp_err_t http_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, HTML_PAGE, strlen(HTML_PAGE));
    return ESP_OK;
}

static esp_err_t http_api_status_handler(httpd_req_t *req)
{
    char response[512];

    wifi_system_status_t status;
    wifi_config_get_status(&status);

    const char *mode_str = "OFF";
    switch (status.current_mode) {
        case WIFI_CONFIG_MODE_AP: mode_str = "AP"; break;
        case WIFI_CONFIG_MODE_STA: mode_str = "STA"; break;
        case WIFI_CONFIG_MODE_APSTA: mode_str = "AP+STA"; break;
        default: break;
    }

    const char *state_str = "DISCONNECTED";
    switch (status.sta_state) {
        case WIFI_STATE_CONNECTING: state_str = "CONNECTING"; break;
        case WIFI_STATE_CONNECTED: state_str = "CONNECTED"; break;
        case WIFI_STATE_GOT_IP: state_str = "GOT_IP"; break;
        case WIFI_STATE_CONNECTION_FAILED: state_str = "FAILED"; break;
        default: break;
    }

    char ip_str[16] = "-";
    if (status.sta_state == WIFI_STATE_GOT_IP) {
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&status.sta_ip_info.ip));
    }

    snprintf(response, sizeof(response),
             "{\"mode\":\"%s\",\"state\":\"%s\",\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"clients\":%lu}",
             mode_str, state_str, status.connected_ssid, ip_str, status.current_rssi,
             (unsigned long)status.connected_clients);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static esp_err_t http_api_scan_handler(httpd_req_t *req)
{
    if (req->method == HTTP_POST) {
        // Start scan
        wifi_config_start_scan();
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"status\":\"scanning\"}", -1);
        return ESP_OK;
    }

    // GET - return scan results
    char *response = malloc(HTTP_RESPONSE_SIZE);
    if (!response) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int offset = sprintf(response, "{\"scanning\":%s,\"networks\":[",
                         s_scan_in_progress ? "true" : "false");

    if (!s_scan_in_progress && s_scan_results) {
        for (int i = 0; i < s_scan_result_count && offset < HTTP_RESPONSE_SIZE - 100; i++) {
            if (i > 0) offset += sprintf(response + offset, ",");
            offset += snprintf(response + offset, HTTP_RESPONSE_SIZE - offset,
                              "{\"ssid\":\"%s\",\"rssi\":%d,\"channel\":%d,\"auth\":\"%s\"}",
                              s_scan_results[i].ssid, s_scan_results[i].rssi,
                              s_scan_results[i].channel,
                              wifi_config_authmode_str(s_scan_results[i].authmode));
        }
    }

    strcat(response, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    free(response);
    return ESP_OK;
}

static esp_err_t http_api_connect_handler(httpd_req_t *req)
{
    if (req->method == HTTP_DELETE) {
        wifi_config_disconnect();
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"message\":\"Disconnected\"}", -1);
        return ESP_OK;
    }

    // POST - connect to network
    char content[256] = {0};
    int received = httpd_req_recv(req, content, sizeof(content) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }

    // Simple JSON parsing
    char ssid[64] = {0}, password[64] = {0};
    char *ssid_start = strstr(content, "\"ssid\":\"");
    char *pass_start = strstr(content, "\"password\":\"");

    if (ssid_start) {
        ssid_start += 8;
        char *ssid_end = strchr(ssid_start, '"');
        if (ssid_end) {
            int len = ssid_end - ssid_start;
            if (len > 0 && len < sizeof(ssid)) {
                strncpy(ssid, ssid_start, len);
            }
        }
    }

    if (pass_start) {
        pass_start += 12;
        char *pass_end = strchr(pass_start, '"');
        if (pass_end) {
            int len = pass_end - pass_start;
            if (len >= 0 && len < sizeof(password)) {
                strncpy(password, pass_start, len);
            }
        }
    }

    if (strlen(ssid) == 0) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"error\":\"SSID required\"}", -1);
        return ESP_OK;
    }

    // Add network and connect
    wifi_config_add_network(ssid, password, 0);
    wifi_config_connect(ssid, password);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"message\":\"Connecting...\"}", -1);
    return ESP_OK;
}

static esp_err_t http_api_config_handler(httpd_req_t *req)
{
    if (req->method == HTTP_DELETE) {
        wifi_config_reset_defaults();
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"message\":\"Reset to defaults\"}", -1);
        return ESP_OK;
    }

    // POST - save config
    char content[256] = {0};
    int received = httpd_req_recv(req, content, sizeof(content) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }

    // Parse AP config
    char ap_ssid[64] = {0}, ap_password[64] = {0};
    char *ssid_start = strstr(content, "\"ap_ssid\":\"");
    char *pass_start = strstr(content, "\"ap_password\":\"");

    if (ssid_start) {
        ssid_start += 11;
        char *ssid_end = strchr(ssid_start, '"');
        if (ssid_end) {
            int len = ssid_end - ssid_start;
            if (len > 0 && len < sizeof(ap_ssid)) {
                strncpy(ap_ssid, ssid_start, len);
            }
        }
    }

    if (pass_start) {
        pass_start += 15;
        char *pass_end = strchr(pass_start, '"');
        if (pass_end) {
            int len = pass_end - pass_start;
            if (len >= 0 && len < sizeof(ap_password)) {
                strncpy(ap_password, pass_start, len);
            }
        }
    }

    wifi_config_set_ap_config(strlen(ap_ssid) > 0 ? ap_ssid : NULL,
                              ap_password, 0);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"message\":\"AP config saved\"}", -1);
    return ESP_OK;
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WIFI_CONFIG_HTTP_PORT;
    config.lru_purge_enable = true;
    config.max_uri_handlers = 8;

    esp_err_t ret = httpd_start(&s_http_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register URI handlers
    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = http_get_handler
    };
    httpd_register_uri_handler(s_http_server, &uri_root);

    httpd_uri_t uri_status = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = http_api_status_handler
    };
    httpd_register_uri_handler(s_http_server, &uri_status);

    httpd_uri_t uri_scan_get = {
        .uri = "/api/scan",
        .method = HTTP_GET,
        .handler = http_api_scan_handler
    };
    httpd_register_uri_handler(s_http_server, &uri_scan_get);

    httpd_uri_t uri_scan_post = {
        .uri = "/api/scan",
        .method = HTTP_POST,
        .handler = http_api_scan_handler
    };
    httpd_register_uri_handler(s_http_server, &uri_scan_post);

    httpd_uri_t uri_connect_post = {
        .uri = "/api/connect",
        .method = HTTP_POST,
        .handler = http_api_connect_handler
    };
    httpd_register_uri_handler(s_http_server, &uri_connect_post);

    httpd_uri_t uri_connect_delete = {
        .uri = "/api/connect",
        .method = HTTP_DELETE,
        .handler = http_api_connect_handler
    };
    httpd_register_uri_handler(s_http_server, &uri_connect_delete);

    httpd_uri_t uri_config_post = {
        .uri = "/api/config",
        .method = HTTP_POST,
        .handler = http_api_config_handler
    };
    httpd_register_uri_handler(s_http_server, &uri_config_post);

    httpd_uri_t uri_config_delete = {
        .uri = "/api/config",
        .method = HTTP_DELETE,
        .handler = http_api_config_handler
    };
    httpd_register_uri_handler(s_http_server, &uri_config_delete);

    ESP_LOGI(TAG, "HTTP server started on port %d", WIFI_CONFIG_HTTP_PORT);
    return ESP_OK;
}

static void stop_http_server(void)
{
    if (s_http_server) {
        httpd_stop(s_http_server);
        s_http_server = NULL;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}
