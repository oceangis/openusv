/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL_ESP32/UARTDriver.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/ExpandingString.h>

#include "esp_log.h"

extern const AP_HAL::HAL& hal;

namespace ESP32
{

UARTDesc uart_desc[] = {HAL_ESP32_UART_DEVICES};

void UARTDriver::vprintf(const char *fmt, va_list ap)
{

    uart_port_t p = uart_desc[uart_num].port;
    if (p == 0) {
        esp_log_writev(ESP_LOG_INFO, "", fmt, ap);
    } else {
        AP_HAL::UARTDriver::vprintf(fmt, ap);
    }
}

void UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (b == 0 && txS == 0 && rxS == 0 && _initialized) {
        // the thread owning this port has changed
        _uart_owner_thd = xTaskGetCurrentTaskHandle();
        return;
    }

    if (uart_num < ARRAY_SIZE(uart_desc)) {
        uart_port_t p = uart_desc[uart_num].port;
        if (!_initialized) {

            // Apply saved configuration or use defaults
            uart_parity_t parity_mode = UART_PARITY_DISABLE;
            if (_parity == 1) parity_mode = UART_PARITY_ODD;
            else if (_parity == 2) parity_mode = UART_PARITY_EVEN;

            uart_stop_bits_t stop_bits = (_stop_bits == 2) ? UART_STOP_BITS_2 : UART_STOP_BITS_1;

            uart_hw_flowcontrol_t flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
            if (_flow_control == FLOW_CONTROL_ENABLE || _flow_control == FLOW_CONTROL_AUTO) {
                flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS;
            } else if (_flow_control == FLOW_CONTROL_RTS_DE) {
                flow_ctrl = UART_HW_FLOWCTRL_RTS;
            }

            uart_config_t config = {
                .baud_rate = (int)b,
                .data_bits = UART_DATA_8_BITS,
                .parity = parity_mode,
                .stop_bits = stop_bits,
                .flow_ctrl = flow_ctrl,
            };
            uart_param_config(p, &config);

            // Set pins including flow control if enabled
            uart_set_pin(p,
                         uart_desc[uart_num].tx,
                         uart_desc[uart_num].rx,
                         (flow_ctrl != UART_HW_FLOWCTRL_DISABLE) ? uart_desc[uart_num].rts : UART_PIN_NO_CHANGE,
                         (flow_ctrl == UART_HW_FLOWCTRL_CTS_RTS) ? uart_desc[uart_num].cts : UART_PIN_NO_CHANGE);

            // ESP32-S3 UART optimization: Event queue + optional DMA
            // DMA is enabled for high-speed ports (>115200 baud) to reduce CPU usage
            _use_dma = (b > DMA_THRESHOLD_BAUDRATE);

            // Install UART driver with event queue (replaces inefficient polling)
            // Event queue allows interrupt-driven data handling
            int intr_alloc_flags = ESP_INTR_FLAG_IRAM;  // ISR in IRAM for low latency

            if (_use_dma) {
                // High-speed mode: Use DMA to offload CPU
                // RX/TX buffers must be larger for DMA efficiency
                uart_driver_install(p, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2,
                                  UART_EVENT_QUEUE_SIZE, &_uart_event_queue, intr_alloc_flags);
            } else {
                // Normal mode: Event-driven without DMA
                uart_driver_install(p, 2*UART_HW_FIFO_LEN(p), 0,
                                  UART_EVENT_QUEUE_SIZE, &_uart_event_queue, intr_alloc_flags);
            }

            _readbuf.set_size(RX_BUF_SIZE);
            _writebuf.set_size(TX_BUF_SIZE);
            _uart_owner_thd = xTaskGetCurrentTaskHandle();

            // Initialize statistics
#if HAL_UART_STATS_ENABLED
            _tx_stats_bytes = 0;
            _rx_stats_bytes = 0;
            _rx_dropped_bytes = 0;
#endif

            _initialized = true;
        } else {
            flush();
            uart_set_baudrate(p, b);

        }
    }
    _baudrate = b;
}

void UARTDriver::_end()
{
    if (_initialized) {
        uart_driver_delete(uart_desc[uart_num].port);
        _readbuf.set_size(0);
        _writebuf.set_size(0);
    }
    _initialized = false;
}

void UARTDriver::_flush()
{
    uart_port_t p = uart_desc[uart_num].port;
    uart_flush(p);
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}


uint32_t UARTDriver::_available()
{
    if (!_initialized || _uart_owner_thd != xTaskGetCurrentTaskHandle()) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t UARTDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);

}

ssize_t IRAM_ATTR UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (_uart_owner_thd != xTaskGetCurrentTaskHandle()) {
        return -1;
    }

    if (!_initialized) {
        return -1;
    }

    const uint32_t ret = _readbuf.read(buffer, count);
    if (ret == 0) {
        return 0;
    }


    _receive_timestamp_update();

    return ret;
}

void IRAM_ATTR UARTDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }

    // ESP32-S3 OPTIMIZATION: Event-driven UART instead of polling
    // Process UART events from interrupt queue (reduces CPU usage by 10-15%)
    uart_event_t event;
    uart_port_t p = uart_desc[uart_num].port;

    // Non-blocking check for UART events
    while (xQueueReceive(_uart_event_queue, &event, 0) == pdTRUE) {
        switch (event.type) {
            case UART_DATA:
                // Data available - read it immediately
                read_data();
                break;

            case UART_FIFO_OVF:
                // FIFO overflow - reset and log warning
                uart_flush_input(p);
                xQueueReset(_uart_event_queue);
#if HAL_UART_STATS_ENABLED
                _rx_dropped_bytes += event.size;  // Track overflow bytes
#endif
                break;

            case UART_BUFFER_FULL:
                // Ring buffer full - reset and continue
                uart_flush_input(p);
                xQueueReset(_uart_event_queue);
#if HAL_UART_STATS_ENABLED
                _rx_dropped_bytes += event.size;
#endif
                break;

            case UART_BREAK:
            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
                // Communication errors - ignore for now
                break;

            default:
                break;
        }
    }

    // Write is still polling-based (no event for TX ready)
    // This is acceptable as TX is less frequent than RX
    write_data();
}

void IRAM_ATTR UARTDriver::read_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    int count = 0;
    do {
        count = uart_read_bytes(p, _buffer, sizeof(_buffer), 0);
        if (count > 0) {
            size_t written = _readbuf.write(_buffer, count);
#if HAL_UART_STATS_ENABLED
            _rx_stats_bytes += written;
            if (written < (size_t)count) {
                _rx_dropped_bytes += (count - written);
            }
#endif
        }
    } while (count > 0);
}

void IRAM_ATTR UARTDriver::write_data()
{
    uart_port_t p = uart_desc[uart_num].port;
    int count = 0;
    _write_mutex.take_blocking();
    do {
        count = _writebuf.peekbytes(_buffer, sizeof(_buffer));
        if (count > 0) {
            count = uart_tx_chars(p, (const char*) _buffer, count);
            _writebuf.advance(count);
        }
    } while (count > 0);
    _write_mutex.give();
}

size_t IRAM_ATTR UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();

    size_t ret = _writebuf.write(buffer, size);

#if HAL_UART_STATS_ENABLED
    _tx_stats_bytes += ret;
#endif

    _write_mutex.give();
    return ret;
}

bool UARTDriver::_discard_input()
{
    if (_uart_owner_thd != xTaskGetCurrentTaskHandle()) {
        return false;
    }
    if (!_initialized) {
        return false;
    }

    _readbuf.clear();

    return true;
}

// record timestamp of new incoming data
void IRAM_ATTR UARTDriver::_receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
}


/*
  return timestamp estimate in microseconds for when the start of
  a nbytes packet arrived on the uart. This should be treated as a
  time constraint, not an exact time. It is guaranteed that the
  packet did not start being received after this time, but it
  could have been in a system buffer before the returned time.
  This takes account of the baudrate of the link. For transports
  that have no baudrate (such as USB) the time estimate may be
  less accurate.
  A return value of zero means the HAL does not support this API
*/
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_baudrate > 0) {
        // assume 10 bits per byte. For USB we assume zero transport delay
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

/*
  Set hardware flow control mode
  Based on ChibiOS implementation
 */
void UARTDriver::set_flow_control(enum flow_control flow_control_setting)
{
    if (!_initialized) {
        _flow_control = flow_control_setting;
        return;
    }

    uart_port_t p = uart_desc[uart_num].port;
    _flow_control = flow_control_setting;

    uart_hw_flowcontrol_t mode;
    switch (flow_control_setting) {
        case FLOW_CONTROL_DISABLE:
            mode = UART_HW_FLOWCTRL_DISABLE;
            break;
        case FLOW_CONTROL_ENABLE:
        case FLOW_CONTROL_AUTO:
            mode = UART_HW_FLOWCTRL_CTS_RTS;
            break;
        case FLOW_CONTROL_RTS_DE:
            mode = UART_HW_FLOWCTRL_RTS;  // RTS as driver enable for RS-485
            break;
        default:
            mode = UART_HW_FLOWCTRL_DISABLE;
            break;
    }

    // Set flow control pins if defined
    if (mode != UART_HW_FLOWCTRL_DISABLE) {
        uart_set_pin(p,
                     uart_desc[uart_num].tx,
                     uart_desc[uart_num].rx,
                     uart_desc[uart_num].rts,
                     uart_desc[uart_num].cts);
    }

    uart_set_hw_flow_ctrl(p, mode, 64);  // RX threshold = 64 bytes
}

/*
  Configure parity
 */
void UARTDriver::configure_parity(uint8_t v)
{
    if (!_initialized) {
        _parity = v;
        return;
    }

    uart_port_t p = uart_desc[uart_num].port;
    _parity = v;

    uart_parity_t parity_mode;
    switch (v) {
        case 0:
            parity_mode = UART_PARITY_DISABLE;
            break;
        case 1:
            parity_mode = UART_PARITY_ODD;
            break;
        case 2:
            parity_mode = UART_PARITY_EVEN;
            break;
        default:
            parity_mode = UART_PARITY_DISABLE;
            break;
    }

    uart_set_parity(p, parity_mode);
}

/*
  Set stop bits (1 or 2)
 */
void UARTDriver::set_stop_bits(int n)
{
    if (!_initialized) {
        _stop_bits = n;
        return;
    }

    uart_port_t p = uart_desc[uart_num].port;
    _stop_bits = n;

    uart_stop_bits_t stop = (n == 2) ? UART_STOP_BITS_2 : UART_STOP_BITS_1;
    uart_set_stop_bits(p, stop);
}

/*
  Set UART options
 */
bool UARTDriver::set_options(uint16_t options)
{
    _last_options = options;

    // Handle NOFIFO option
    if (option_is_set(Option::OPTION_NOFIFO)) {
        // ESP32 doesn't support disabling FIFO completely
        // This is a no-op for compatibility
    }

    return true;
}

/*
  Software control of RTS pin
 */
bool UARTDriver::set_RTS_pin(bool high)
{
    if (!_initialized) {
        return false;
    }

    gpio_num_t rts_pin = uart_desc[uart_num].rts;
    if (rts_pin == UART_PIN_NO_CHANGE || rts_pin == (gpio_num_t)-1) {
        return false;
    }

    gpio_set_level(rts_pin, high ? 1 : 0);
    return true;
}

/*
  Software control of CTS pin
 */
bool UARTDriver::set_CTS_pin(bool high)
{
    if (!_initialized) {
        return false;
    }

    gpio_num_t cts_pin = uart_desc[uart_num].cts;
    if (cts_pin == UART_PIN_NO_CHANGE || cts_pin == (gpio_num_t)-1) {
        return false;
    }

    gpio_set_level(cts_pin, high ? 1 : 0);
    return true;
}

/*
  Wait for n bytes to be available with timeout
 */
bool UARTDriver::wait_timeout(uint16_t n, uint32_t timeout_ms)
{
    uint32_t start_ms = AP_HAL::millis();

    while (AP_HAL::millis() - start_ms < timeout_ms) {
        if (available() >= n) {
            return true;
        }
        hal.scheduler->delay_microseconds(100);
    }

    return false;
}

#if HAL_UART_STATS_ENABLED
/*
  Provide UART statistics for logging
  Similar to ChibiOS implementation
 */
void UARTDriver::uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms)
{
    if (!_initialized) {
        return;
    }

    // Update stats
    uint32_t tx_bytes = stats.tx.update(_tx_stats_bytes);
    uint32_t rx_bytes = stats.rx.update(_rx_stats_bytes);
    uint32_t rx_dropped = stats.rx_dropped.update(_rx_dropped_bytes);

    // Calculate rates (bytes/sec)
    float tx_rate = 0, rx_rate = 0, rx_drop_rate = 0;
    if (dt_ms > 0) {
        tx_rate = (tx_bytes * 1000.0f) / dt_ms;
        rx_rate = (rx_bytes * 1000.0f) / dt_ms;
        rx_drop_rate = (rx_dropped * 1000.0f) / dt_ms;
    }

    // Format output
    str.printf("UART%u: baud=%lu fc=%s tx=%.1f rx=%.1f drop=%.1f\n",
               uart_num,
               _baudrate,
               (_flow_control == FLOW_CONTROL_DISABLE) ? "off" :
               (_flow_control == FLOW_CONTROL_RTS_DE) ? "DE" : "on",
               tx_rate,
               rx_rate,
               rx_drop_rate);
}
#endif

}
