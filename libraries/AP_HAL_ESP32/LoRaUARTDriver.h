/*
 * LoRa UART Driver for ESP32-S3
 *
 * Wraps LoRa MAVLink interface as a virtual UART for ArduPilot serial system.
 * Allows LoRa to be used as SERIAL4 for MAVLink telemetry.
 */

#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_ESP32/Semaphores.h>

// C interface for lora_mavlink
extern "C" {
    #include "lora_mavlink.h"
}

namespace ESP32
{

class LoRaUARTDriver : public AP_HAL::UARTDriver
{
public:
    LoRaUARTDriver();
    virtual ~LoRaUARTDriver() = default;

    // UARTDriver interface
    void vprintf(const char *fmt, va_list ap) override;
    bool is_initialized() override;
    bool tx_pending() override;
    uint32_t txspace() override;
    void _timer_tick(void) override;

    uint32_t bw_in_bytes_per_second() const override {
        // LoRa SF8/125kHz theoretical ~1.2kbps, practical ~800bps
        return 800;
    }

    uint64_t receive_time_constraint_us(uint16_t nbytes) override;
    uint32_t get_baud_rate() const override { return _virtual_baudrate; }

    // Flow control (not applicable to LoRa)
    void set_flow_control(enum flow_control flow_control_setting) override {}
    enum flow_control get_flow_control(void) override { return FLOW_CONTROL_DISABLE; }

    // Configuration (mostly ignored for LoRa)
    void configure_parity(uint8_t v) override {}
    void set_stop_bits(int n) override {}
    bool set_options(uint16_t options) override { return true; }
    uint16_t get_options(void) const override { return 0; }

    bool set_RTS_pin(bool high) override { return false; }
    bool set_CTS_pin(bool high) override { return false; }

    bool wait_timeout(uint16_t n, uint32_t timeout_ms) override;

    // LoRa specific methods
    void get_lora_stats(lora_stats_t *stats);
    bool set_frequency(uint32_t freq_hz);
    bool set_tx_power(int8_t power_dbm);

protected:
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    bool _discard_input() override;

private:
    bool _initialized;
    uint32_t _virtual_baudrate;
    uint64_t _last_rx_timestamp;
    Semaphore _write_mutex;
};

}  // namespace ESP32
