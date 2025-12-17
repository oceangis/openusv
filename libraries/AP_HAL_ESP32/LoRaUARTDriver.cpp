/*
 * LoRa UART Driver for ESP32-S3
 *
 * Implementation of virtual UART over LoRa for MAVLink telemetry.
 */

#include "LoRaUARTDriver.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <stdarg.h>

extern const AP_HAL::HAL& hal;

namespace ESP32
{

LoRaUARTDriver::LoRaUARTDriver()
    : AP_HAL::UARTDriver()
    , _initialized(false)
    , _virtual_baudrate(9600)
    , _last_rx_timestamp(0)
{
}

void LoRaUARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    // LoRa is already initialized in main.c before ArduPilot starts
    // Just check if it's ready
    _virtual_baudrate = b;

    if (lora_mavlink_is_initialized()) {
        _initialized = true;
        hal.console->printf("LoRaUARTDriver: Ready (virtual baud %u)\n", (unsigned)b);
    } else {
        hal.console->printf("LoRaUARTDriver: LoRa not initialized\n");
        _initialized = false;
    }
}

void LoRaUARTDriver::_end()
{
    _initialized = false;
}

void LoRaUARTDriver::_flush()
{
    // LoRa handles its own flushing via FreeRTOS task
}

uint32_t LoRaUARTDriver::_available()
{
    if (!_initialized) {
        return 0;
    }
    return lora_mavlink_available();
}

ssize_t LoRaUARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized || buffer == nullptr || count == 0) {
        return 0;
    }

    size_t read_bytes = lora_mavlink_read(buffer, count);

    if (read_bytes > 0) {
        _last_rx_timestamp = AP_HAL::micros64();
    }

    return read_bytes;
}

size_t LoRaUARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized || buffer == nullptr || size == 0) {
        return 0;
    }

    WITH_SEMAPHORE(_write_mutex);

    return lora_mavlink_write(buffer, size);
}

bool LoRaUARTDriver::_discard_input()
{
    if (!_initialized) {
        return false;
    }

    // Read and discard all available data
    uint8_t discard_buf[64];
    while (lora_mavlink_available() > 0) {
        lora_mavlink_read(discard_buf, sizeof(discard_buf));
    }

    return true;
}

void LoRaUARTDriver::vprintf(const char *fmt, va_list ap)
{
    if (!_initialized) {
        return;
    }

    char buf[128];
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n > 0) {
        _write((const uint8_t *)buf, MIN((size_t)n, sizeof(buf)-1));
    }
}

bool LoRaUARTDriver::is_initialized()
{
    return _initialized && lora_mavlink_is_initialized();
}

bool LoRaUARTDriver::tx_pending()
{
    if (!_initialized) {
        return false;
    }
    // Check if TX buffer has data waiting to be sent
    return lora_mavlink_tx_free() < LORA_TX_BUFFER_SIZE;
}

uint32_t LoRaUARTDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    return lora_mavlink_tx_free();
}

void LoRaUARTDriver::_timer_tick(void)
{
    // LoRa uses its own FreeRTOS task for processing
    // No additional timer tick needed
}

uint64_t LoRaUARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    // LoRa has significant latency due to air time
    // Return conservative estimate
    if (_last_rx_timestamp == 0) {
        return 0;
    }

    // Estimate based on LoRa air time (SF8, 125kHz ~10ms per packet)
    uint64_t byte_time_us = 1000000ULL / bw_in_bytes_per_second();
    return _last_rx_timestamp + (nbytes * byte_time_us);
}

bool LoRaUARTDriver::wait_timeout(uint16_t n, uint32_t timeout_ms)
{
    if (!_initialized) {
        return false;
    }

    uint32_t start_ms = AP_HAL::millis();

    while ((AP_HAL::millis() - start_ms) < timeout_ms) {
        if (_available() >= n) {
            return true;
        }
        hal.scheduler->delay(1);
    }

    return false;
}

void LoRaUARTDriver::get_lora_stats(lora_stats_t *stats)
{
    if (stats) {
        lora_mavlink_get_stats(stats);
    }
}

bool LoRaUARTDriver::set_frequency(uint32_t freq_hz)
{
    return lora_mavlink_set_frequency(freq_hz);
}

bool LoRaUARTDriver::set_tx_power(int8_t power_dbm)
{
    return lora_mavlink_set_tx_power(power_dbm);
}

}  // namespace ESP32
