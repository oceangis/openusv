# Thread Optimization Recommendations
## ESP32-S3 IDF Project - Specific Code Changes

**Based on**: ChibiOS HAL Analysis
**Priority**: Low to Medium (no critical issues)

---

## Optimization 1: WiFi HTTP Handler Timeout Protection

### Issue Identified
HTTP request handlers could potentially block if processing large requests or during NVS write operations.

### ChibiOS Reference Pattern
ChibiOS networking code uses timeouts and non-blocking patterns extensively to prevent handler blocking.

### Target File
`components/wifi_config/src/wifi_config.c`

### Current Implementation
```c
// Line 1450: Config save handler
static esp_err_t http_api_config_handler(httpd_req_t *req)
{
    // ... parsing ...
    wifi_config_set_ap_config(strlen(ap_ssid) > 0 ? ap_ssid : NULL,
                              ap_password, 0);
    // Could block during NVS write
}
```

### Recommended Change
```c
// Add timing monitoring
static esp_err_t http_api_config_handler(httpd_req_t *req)
{
    uint32_t start_ms = get_time_ms();

    if (req->method == HTTP_DELETE) {
        wifi_config_reset_defaults();
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"message\":\"Reset to defaults\"}", -1);
        goto done;
    }

    // ... existing parsing code ...

    wifi_config_set_ap_config(strlen(ap_ssid) > 0 ? ap_ssid : NULL,
                              ap_password, 0);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"message\":\"AP config saved\"}", -1);

done:
    uint32_t elapsed = get_time_ms() - start_ms;
    if (elapsed > 50) {
        ESP_LOGW(TAG, "HTTP handler took %lu ms (threshold: 50ms)", elapsed);
    }
    return ESP_OK;
}
```

### Expected Benefit
- Early detection of handler blocking issues
- Metrics for performance tuning
- No functional impact, monitoring only

### Risk Assessment
- **Risk**: None - monitoring only
- **Testing**: Monitor logs during web configuration

---

## Optimization 2: LoRa Task Priority Alignment

### Issue Identified
LoRa task uses FreeRTOS priority 5, while ArduPilot PRIORITY_SPI tasks typically use higher priority for time-critical SPI operations.

### ChibiOS Reference Pattern
```cpp
// All ExternalAHRS backends use PRIORITY_SPI
hal.scheduler->thread_create(
    FUNCTOR_BIND_MEMBER(&update_thread, void),
    "NAME", 2048,
    AP_HAL::Scheduler::PRIORITY_SPI,  // High priority for I/O
    0
);
```

### Target File
`components/lora_mavlink/src/lora_mavlink.c`

### Current Implementation
```c
// Line 410: Task creation
BaseType_t ret = xTaskCreate(
    lora_task,
    "lora_task",
    4096,
    NULL,
    5,  // Priority 5 (medium)
    &lora_ctx.task_handle
);
```

### Recommended Change
```c
// Line 410: Increase priority to match SPI I/O tasks
#define LORA_TASK_PRIORITY  (configMAX_PRIORITIES - 3)  // High, but below critical

BaseType_t ret = xTaskCreate(
    lora_task,
    "lora_task",
    4096,
    NULL,
    LORA_TASK_PRIORITY,  // Higher priority for RF timing
    &lora_ctx.task_handle
);
```

### Expected Benefit
- Reduced latency for LoRa packet transmission
- Better RF timing consistency
- Aligns with ArduPilot's I/O priority scheme

### Risk Assessment
- **Risk**: Low - LoRa task is well-designed and won't monopolize CPU
- **Testing**: Monitor LoRa packet loss rate and latency

---

## Optimization 3: NVS Write Deferral for WiFi Config

### Issue Identified
WiFi configuration saves trigger immediate NVS writes, which can block for several milliseconds.

### ChibiOS Reference Pattern
ChibiOS storage subsystem uses background thread for Flash writes to avoid blocking main operations.

### Target File
`components/wifi_config/src/wifi_config.c`

### Current Implementation
```c
// Line 779: Synchronous NVS write
static esp_err_t save_config_to_nvs(void)
{
    // ... open NVS ...
    ret = nvs_set_blob(handle, NVS_KEY_CONFIG, &s_config, sizeof(s_config));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);  // Can block for ~10ms
    }
    nvs_close(handle);
    return ret;
}
```

### Recommended Change (Option A - Simple)
```c
// Add deferred write flag
static bool s_config_dirty = false;

static esp_err_t save_config_to_nvs_deferred(void)
{
    s_config_dirty = true;
    return ESP_OK;  // Mark for later save
}

// Call from periodic task (1Hz)
static void periodic_save_task(void)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Check every 1 second

        if (s_config_dirty) {
            if (xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                esp_err_t ret = save_config_to_nvs();  // Actual write
                if (ret == ESP_OK) {
                    s_config_dirty = false;
                }
                xSemaphoreGive(s_config_mutex);
            }
        }
    }
}
```

### Recommended Change (Option B - Use External Storage Callback)
```c
// WiFi config already has this infrastructure!
// Just use the AP_Param callback that saves to Flash partition 0x45

// In wifi_config_system_init():
wifi_config_register_save_callback(wifi_config_save_to_ap_param);
// This already routes saves through AP_Param which handles deferred writes
```

### Expected Benefit
- Eliminates blocking NVS writes from web handlers
- Better HTTP response times
- Uses existing AP_Param infrastructure (Option B)

### Risk Assessment
- **Risk**: Low - config changes are infrequent
- **Recommendation**: Use Option B (already implemented!)
- **Testing**: Verify web config changes persist across reboots

---

## Optimization 4: BNO08x Thread Health Monitoring

### Issue Identified
BNO08x thread could stall on I2C errors without detection.

### ChibiOS Reference Pattern
ChibiOS threads typically include watchdog monitoring and error recovery.

### Target File
`libraries/AP_ExternalAHRS/AP_ExternalAHRS_BNO08x.cpp`

### Current Implementation
```cpp
// Thread runs forever with no health monitoring
void AP_ExternalAHRS_BNO08x::update_thread()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(10);
    }

    if (!init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Thread init failed");
        return;  // Thread exits silently
    }

    while (true) {
        // Read sensor...
        hal.scheduler->delay(10);
    }
}
```

### Recommended Change
```cpp
// Add thread health heartbeat
void AP_ExternalAHRS_BNO08x::update_thread()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(10);
    }

    if (!init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "BNO08x: Thread init failed");
        // Stay in loop instead of exiting
        while (true) {
            hal.scheduler->delay(5000);
            // Retry init periodically
            if (init()) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BNO08x: Thread recovered");
                break;
            }
        }
    }

    uint32_t last_heartbeat_ms = AP_HAL::millis();
    uint32_t error_count = 0;

    while (true) {
        uint16_t packet_length = 0;
        uint32_t timestamp_us = 0;
        bool got_packet = false;

        {
            WITH_SEMAPHORE(dev->get_semaphore());
            got_packet = shtp_receive(rx_buffer, packet_length, &timestamp_us);
        }

        if (got_packet) {
            process_packet(rx_buffer, packet_length);
            last_heartbeat_ms = AP_HAL::millis();
            error_count = 0;
        } else {
            // Check for thread stall
            uint32_t now = AP_HAL::millis();
            if (now - last_heartbeat_ms > 5000) {
                error_count++;
                if (error_count == 1) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                 "BNO08x: Thread stalled for 5s");
                }
                if (error_count > 10) {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                                 "BNO08x: Thread recovery attempt");
                    // Attempt recovery
                    sensor_initialized = false;
                    if (init()) {
                        error_count = 0;
                        last_heartbeat_ms = now;
                    }
                }
            }
        }

        if (have_rotation) {
            WITH_SEMAPHORE(data_sem);
            // Copy data...
        }

        hal.scheduler->delay(10);
    }
}
```

### Expected Benefit
- Automatic thread recovery on I2C errors
- Early detection of sensor failures
- Better system resilience

### Risk Assessment
- **Risk**: Low - adds robustness
- **Testing**: Simulate I2C errors (disconnect sensor)

---

## Optimization 5: LoRa Ring Buffer Overflow Protection

### Issue Identified
LoRa TX buffer can overflow if data is written faster than RF transmission.

### ChibiOS Reference Pattern
ChibiOS ring buffers typically include overflow warnings and flow control.

### Target File
`components/lora_mavlink/src/lora_mavlink.c`

### Current Implementation
```c
// Line 455: Public write API
size_t lora_mavlink_write(const uint8_t *data, size_t len)
{
    if (!lora_ctx.initialized || !data || len == 0) {
        return 0;
    }

    if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return 0;
    }

    size_t written = ring_buffer_write(&lora_ctx.tx_buffer, data, len);
    xSemaphoreGive(lora_ctx.mutex);

    return written;  // May be less than len if buffer full
}
```

### Recommended Change
```c
// Add overflow statistics
typedef struct {
    uint32_t tx_packets;
    uint32_t rx_packets;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t crc_errors;
    uint32_t tx_overflow_bytes;  // NEW
    uint32_t rx_overflow_bytes;  // NEW
    int16_t last_rssi;
    int8_t last_snr;
} lora_stats_t;

// Update write function
size_t lora_mavlink_write(const uint8_t *data, size_t len)
{
    if (!lora_ctx.initialized || !data || len == 0) {
        return 0;
    }

    if (xSemaphoreTake(lora_ctx.mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return 0;
    }

    size_t written = ring_buffer_write(&lora_ctx.tx_buffer, data, len);

    if (written < len) {
        size_t lost = len - written;
        lora_ctx.stats.tx_overflow_bytes += lost;

        // Log first overflow and every 100th
        static uint32_t overflow_count = 0;
        if (overflow_count == 0 || overflow_count % 100 == 0) {
            ESP_LOGW(TAG, "TX buffer overflow: lost %d bytes (total: %lu)",
                     (int)lost, lora_ctx.stats.tx_overflow_bytes);
        }
        overflow_count++;
    }

    xSemaphoreGive(lora_ctx.mutex);
    return written;
}
```

### Expected Benefit
- Visibility into LoRa throughput limitations
- Early warning of configuration issues
- Better debugging of RF link problems

### Risk Assessment
- **Risk**: None - monitoring only
- **Testing**: Send high-rate telemetry and check stats

---

## Summary of Recommendations

| Optimization | Priority | Complexity | Expected Benefit | Risk |
|-------------|----------|------------|------------------|------|
| 1. WiFi Handler Timeout | Medium | Low | Better monitoring | None |
| 2. LoRa Task Priority | Low | Low | Better RF timing | Low |
| 3. NVS Write Deferral | Low | Low | Already done via AP_Param | None |
| 4. BNO08x Health Monitor | Medium | Medium | Auto-recovery | Low |
| 5. LoRa Overflow Stats | Low | Low | Better diagnostics | None |

---

## Implementation Priority

### Phase 1 (Quick Wins)
1. ✅ Optimization 3 - NVS deferral (already implemented via AP_Param!)
2. Add Optimization 5 - LoRa overflow stats (30 minutes)
3. Add Optimization 1 - HTTP handler timing (30 minutes)

### Phase 2 (Performance)
4. Implement Optimization 2 - LoRa priority (10 minutes)
5. Test and tune based on metrics

### Phase 3 (Robustness)
6. Implement Optimization 4 - BNO08x health monitoring (2 hours)
7. Add comprehensive watchdog system

---

## Testing Plan

### 1. Stress Test - WiFi Configuration
```
- Rapidly change WiFi settings via web interface
- Monitor HTTP handler timing logs
- Verify settings persist across reboots
- Expected: All handlers < 50ms
```

### 2. Stress Test - LoRa Throughput
```
- Send MAVLink data at maximum rate
- Monitor LoRa overflow statistics
- Measure packet loss rate
- Expected: < 1% packet loss at 1Hz telemetry
```

### 3. Fault Injection - BNO08x
```
- Physically disconnect I2C sensor during operation
- Reconnect after 30 seconds
- Verify automatic recovery
- Expected: Recovery within 5 seconds of reconnection
```

### 4. Long-Duration Test
```
- Run system for 24 hours
- Monitor all thread health metrics
- Check for memory leaks (heap fragmentation)
- Expected: Stable operation, no degradation
```

---

## Metrics to Track

### Real-Time Metrics
- Main loop iteration time (target: < 20ms for 50Hz)
- Thread wake-up latency (target: < 1ms)
- Semaphore contention time (target: < 100µs)

### Communication Metrics
- BNO08x packet rate (target: ~100Hz)
- LoRa TX/RX rate (target: ~1Hz)
- WiFi connection stability (target: > 99% uptime)

### Error Metrics
- I2C/SPI timeout count (target: 0)
- Thread stall count (target: 0)
- Buffer overflow events (target: 0)

---

**Generated**: 2025-12-28
**Status**: Ready for Implementation
**Estimated Total Implementation Time**: 4-6 hours
