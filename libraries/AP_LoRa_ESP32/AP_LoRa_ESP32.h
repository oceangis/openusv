/**
 * @file AP_LoRa_ESP32.h
 * @brief ArduPilot LoRa Parameter Integration for ESP32-S3
 *
 * This class integrates LoRa configuration with ArduPilot's AP_Param system,
 * allowing LoRa settings to be configured via MissionPlanner/GCS.
 * Used for MAVLink telemetry over LoRa (SERIAL4).
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_LORA_ESP32_ENABLED
#define AP_LORA_ESP32_ENABLED 1
#endif

#if AP_LORA_ESP32_ENABLED

#include <AP_Param/AP_Param.h>

// Forward declaration of C functions from lora_mavlink
extern "C" {
    #include "lora_mavlink.h"
}

/**
 * @brief LoRa configuration class for ArduPilot parameter system
 *
 * Provides LORA_ prefixed parameters that can be configured through GCS.
 * Parameters are applied to lora_mavlink component at runtime.
 */
class AP_LoRa_ESP32 {
public:
    AP_LoRa_ESP32();

    // Do not allow copies
    CLASS_NO_COPY(AP_LoRa_ESP32);

    // Parameter table
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Initialize LoRa system with current parameters
     */
    void init();

    /**
     * @brief Update method called from main loop
     * Handles parameter changes at runtime
     */
    void update();

    /**
     * @brief Get singleton instance
     */
    static AP_LoRa_ESP32 *get_singleton() { return _singleton; }

    /**
     * @brief Check if LoRa is enabled
     */
    bool is_enabled() const { return _enable.get() > 0; }

    /**
     * @brief Get current frequency in Hz
     */
    uint32_t get_frequency() const { return _freq.get() * 1000; }

    /**
     * @brief Get TX power in dBm
     */
    int8_t get_tx_power() const { return _power.get(); }

    /**
     * @brief Get spreading factor
     */
    uint8_t get_sf() const { return _sf.get(); }

    /**
     * @brief Get bandwidth setting
     */
    uint8_t get_bw() const { return _bw.get(); }

    /**
     * @brief Check if LoRa is initialized
     */
    bool is_initialized() const { return _initialized && lora_mavlink_is_initialized(); }

    /**
     * @brief Get statistics
     */
    void get_stats(lora_stats_t *stats);

    /**
     * @brief Get last RSSI
     */
    int16_t get_rssi() const;

    /**
     * @brief Get last SNR
     */
    int8_t get_snr() const;

private:
    static AP_LoRa_ESP32 *_singleton;

    // Primary enable parameter
    AP_Int8 _enable;            // LORA_ENABLE: 0=disabled, 1=enabled

    // Radio configuration
    AP_Int32 _freq;             // LORA_FREQ: Frequency in kHz (433000=433MHz)
    AP_Int8 _power;             // LORA_POWER: TX power in dBm (2-22)
    AP_Int8 _sf;                // LORA_SF: Spreading factor (6-12)
    AP_Int8 _bw;                // LORA_BW: Bandwidth 0=125kHz, 1=250kHz, 2=500kHz
    AP_Int8 _cr;                // LORA_CR: Coding rate (1=4/5 to 4=4/8)

    // Status tracking
    bool _initialized;
    uint32_t _last_update_ms;

    // Parameter change detection
    int8_t _last_enable;
    int32_t _last_freq;
    int8_t _last_power;
    int8_t _last_sf;
    int8_t _last_bw;
    int8_t _last_cr;

    /**
     * @brief Check if any parameters have changed
     */
    bool params_changed() const;

    /**
     * @brief Apply current parameters to LoRa hardware
     */
    void apply_params();
};

#endif // AP_LORA_ESP32_ENABLED
