/**
 * @file AP_LoRa_ESP32.cpp
 * @brief ArduPilot LoRa Parameter Integration Implementation
 *
 * Provides LORA_ parameters for runtime configuration of LoRa radio.
 * Integrates with lora_mavlink component for MAVLink telemetry.
 */

#include "AP_LoRa_ESP32.h"

#if AP_LORA_ESP32_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_LoRa_ESP32 *AP_LoRa_ESP32::_singleton = nullptr;

// ============================================================================
// Parameter table definition
// ============================================================================

// @Group: LORA_
// @Path: AP_LoRa_ESP32.cpp
const AP_Param::GroupInfo AP_LoRa_ESP32::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: LoRa Enable
    // @Description: Enable or disable LoRa telemetry (SERIAL4)
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_LoRa_ESP32, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: FREQ
    // @DisplayName: LoRa Frequency
    // @Description: Radio frequency in kHz. Common values: 433000 (433MHz), 868000 (868MHz), 915000 (915MHz)
    // @Range: 410000 930000
    // @Units: kHz
    // @User: Standard
    AP_GROUPINFO("FREQ", 2, AP_LoRa_ESP32, _freq, LORA_FREQ_HZ / 1000),

    // @Param: POWER
    // @DisplayName: LoRa TX Power
    // @Description: Transmit power in dBm. Higher power = longer range but more battery use.
    // @Range: 2 22
    // @Units: dBm
    // @User: Standard
    AP_GROUPINFO("POWER", 3, AP_LoRa_ESP32, _power, LORA_TX_POWER),

    // @Param: SF
    // @DisplayName: LoRa Spreading Factor
    // @Description: Spreading factor. Higher SF = longer range but slower data rate. SF8 recommended for MAVLink.
    // @Range: 6 12
    // @User: Advanced
    AP_GROUPINFO("SF", 4, AP_LoRa_ESP32, _sf, LORA_SF),

    // @Param: BW
    // @DisplayName: LoRa Bandwidth
    // @Description: Radio bandwidth. Lower BW = longer range but slower data rate.
    // @Values: 0:125kHz,1:250kHz,2:500kHz
    // @User: Advanced
    AP_GROUPINFO("BW", 5, AP_LoRa_ESP32, _bw, 0),

    // @Param: CR
    // @DisplayName: LoRa Coding Rate
    // @Description: Forward error correction coding rate. Higher = more robust but slower.
    // @Values: 1:4/5,2:4/6,3:4/7,4:4/8
    // @User: Advanced
    AP_GROUPINFO("CR", 6, AP_LoRa_ESP32, _cr, 1),

    AP_GROUPEND
};

// ============================================================================
// Constructor
// ============================================================================

AP_LoRa_ESP32::AP_LoRa_ESP32()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_LoRa_ESP32 must be singleton");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    _initialized = false;
    _last_update_ms = 0;

    // Initialize change detection
    _last_enable = -1;
    _last_freq = -1;
    _last_power = -1;
    _last_sf = -1;
    _last_bw = -1;
    _last_cr = -1;
}

// ============================================================================
// Initialization
// ============================================================================

void AP_LoRa_ESP32::init()
{
    if (_initialized) {
        return;
    }

    if (!is_enabled()) {
        return;
    }

    // LoRa is already initialized in main.c before ArduPilot starts
    // Just verify it's running and apply any parameter overrides
    if (lora_mavlink_is_initialized()) {
        // Apply current parameter values
        apply_params();

        _initialized = true;
        hal.console->printf("AP_LoRa_ESP32: initialized (freq=%u kHz, power=%d dBm, SF%d)\n",
                           (unsigned)_freq.get(), _power.get(), _sf.get());
    } else {
        hal.console->printf("AP_LoRa_ESP32: lora_mavlink not initialized\n");
    }

    // Store initial values for change detection
    _last_enable = _enable.get();
    _last_freq = _freq.get();
    _last_power = _power.get();
    _last_sf = _sf.get();
    _last_bw = _bw.get();
    _last_cr = _cr.get();
}

// ============================================================================
// Update loop
// ============================================================================

void AP_LoRa_ESP32::update()
{
    if (!_initialized) {
        init();
        return;
    }

    // Check for parameter changes (from MissionPlanner)
    if (params_changed()) {
        apply_params();
        hal.console->printf("AP_LoRa_ESP32: params updated\n");
    }
}

// ============================================================================
// Parameter change detection
// ============================================================================

bool AP_LoRa_ESP32::params_changed() const
{
    return (_enable.get() != _last_enable) ||
           (_freq.get() != _last_freq) ||
           (_power.get() != _last_power) ||
           (_sf.get() != _last_sf) ||
           (_bw.get() != _last_bw) ||
           (_cr.get() != _last_cr);
}

// ============================================================================
// Apply parameters to hardware
// ============================================================================

void AP_LoRa_ESP32::apply_params()
{
    // Apply frequency change
    if (_freq.get() != _last_freq) {
        uint32_t freq_hz = _freq.get() * 1000;
        if (lora_mavlink_set_frequency(freq_hz)) {
            hal.console->printf("AP_LoRa_ESP32: frequency set to %u Hz\n", (unsigned)freq_hz);
        }
    }

    // Apply power change
    if (_power.get() != _last_power) {
        if (lora_mavlink_set_tx_power(_power.get())) {
            hal.console->printf("AP_LoRa_ESP32: TX power set to %d dBm\n", _power.get());
        }
    }

    // Note: SF, BW, CR changes require re-initialization
    // For now, log a message indicating reboot is needed
    if (_sf.get() != _last_sf || _bw.get() != _last_bw || _cr.get() != _last_cr) {
        hal.console->printf("AP_LoRa_ESP32: SF/BW/CR changed, reboot required\n");
    }

    // Update change detection values
    _last_enable = _enable.get();
    _last_freq = _freq.get();
    _last_power = _power.get();
    _last_sf = _sf.get();
    _last_bw = _bw.get();
    _last_cr = _cr.get();
}

// ============================================================================
// Status functions
// ============================================================================

void AP_LoRa_ESP32::get_stats(lora_stats_t *stats)
{
    if (stats && _initialized) {
        lora_mavlink_get_stats(stats);
    }
}

int16_t AP_LoRa_ESP32::get_rssi() const
{
    if (!_initialized) {
        return -127;
    }
    lora_stats_t stats;
    lora_mavlink_get_stats(&stats);
    return stats.last_rssi;
}

int8_t AP_LoRa_ESP32::get_snr() const
{
    if (!_initialized) {
        return -128;
    }
    lora_stats_t stats;
    lora_mavlink_get_stats(&stats);
    return stats.last_snr;
}

#endif // AP_LORA_ESP32_ENABLED
