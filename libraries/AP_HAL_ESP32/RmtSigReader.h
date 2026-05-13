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
 *
 */
#pragma once

#include "AP_HAL_ESP32.h"

#ifdef HAL_ESP32_RCIN

// Modern ESP-IDF v5+ RMT RX API (legacy driver/rmt.h has been migrated away from)
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class ESP32::RmtSigReader
{
public:
    static const uint32_t resolution_hz = 1000000;   // 1 MHz tick = 1 us per duration count
    static const size_t   mem_block_symbols = 128;   // RMT memory block size in symbols
    static const size_t   rx_buf_symbols = 128;      // user RX buffer size (symbols)
    static const uint32_t idle_threshold_us = 3000;  // gap > 3ms terminates a frame

    void init();
    bool read(uint32_t &width_high, uint32_t &width_low);
    void disable();

private:
    bool add_item(uint32_t duration, bool level);
    void rearm_rx();

    // ISR callback — copies event data to queue
    static bool IRAM_ATTR on_recv_done(rmt_channel_handle_t channel,
                                       const rmt_rx_done_event_data_t *edata,
                                       void *user_ctx);

    rmt_channel_handle_t rx_chan = nullptr;
    QueueHandle_t recv_queue = nullptr;
    rmt_receive_config_t rx_config = {};

    // Double buffer: one is armed with rmt_receive(), other is being processed
    rmt_symbol_word_t buf_a[rx_buf_symbols];
    rmt_symbol_word_t buf_b[rx_buf_symbols];
    rmt_symbol_word_t *armed_buf = nullptr;    // currently armed (passed to rmt_receive)
    rmt_symbol_word_t *process_buf = nullptr;  // last received, being walked by read()
    size_t process_count = 0;                  // num symbols in process_buf
    size_t process_idx = 0;                    // next symbol index to consume

    // Pulse-pair state — same semantics as legacy driver
    uint32_t last_high = 0;
    uint32_t ready_high = 0;
    uint32_t ready_low = 0;
    bool pulse_ready = false;

    bool initialised = false;
    bool enabled = false;
};

#endif  // HAL_ESP32_RCIN
