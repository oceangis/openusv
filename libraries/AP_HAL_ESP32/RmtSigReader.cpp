/*
 * RmtSigReader — pulse capture for RC input (PPM / SBUS / DSM).
 *
 * Migrated from legacy <driver/rmt.h> to the modern <driver/rmt_rx.h> API
 * available in ESP-IDF v5.x. The legacy driver was deprecated and
 * caused ArduPilot main() to exit early on ESP32-S3 + IDF 5.5.
 *
 * Architecture:
 *   - RX channel runs at 1 MHz resolution (1us per duration count)
 *   - We pass a buffer to rmt_receive(); on completion the ISR callback
 *     fires and the event data (pointing to that buffer) is enqueued.
 *   - read() dequeues the event, swaps to the OTHER buffer, immediately
 *     re-arms rmt_receive() with the now-free buffer, then walks the
 *     just-completed buffer producing (high, low) pulse pairs.
 *   - Double buffering means we never miss a frame just because the
 *     consumer is still processing the previous one.
 */

#include <AP_HAL/HAL.h>
#include "RmtSigReader.h"

#ifdef HAL_ESP32_RCIN

#include "esp_log.h"

using namespace ESP32;

static const char *TAG = "RmtSigReader";

void RmtSigReader::init()
{
    if (initialised) {
        return;
    }

    // ---- Create RX channel ----
    rmt_rx_channel_config_t chan_config = {};
    chan_config.gpio_num = (gpio_num_t)HAL_ESP32_RCIN;
    chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    chan_config.resolution_hz = resolution_hz;        // 1 MHz
    chan_config.mem_block_symbols = mem_block_symbols; // 128
    chan_config.flags.invert_in = false;
    chan_config.flags.with_dma = false;

    esp_err_t err = rmt_new_rx_channel(&chan_config, &rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_rx_channel failed: %s (%d)", esp_err_to_name(err), err);
        return;
    }

    // ---- Create signal queue ----
    recv_queue = xQueueCreate(2, sizeof(rmt_rx_done_event_data_t));
    if (recv_queue == nullptr) {
        ESP_LOGE(TAG, "xQueueCreate failed");
        rmt_del_channel(rx_chan);
        rx_chan = nullptr;
        return;
    }

    // ---- Register ISR callback ----
    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = on_recv_done;
    err = rmt_rx_register_event_callbacks(rx_chan, &cbs, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_rx_register_event_callbacks failed: %s", esp_err_to_name(err));
        vQueueDelete(recv_queue);
        recv_queue = nullptr;
        rmt_del_channel(rx_chan);
        rx_chan = nullptr;
        return;
    }

    // ---- Enable channel ----
    err = rmt_enable(rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(err));
        vQueueDelete(recv_queue);
        recv_queue = nullptr;
        rmt_del_channel(rx_chan);
        rx_chan = nullptr;
        return;
    }

    // ---- Setup receive config ----
    // signal_range_min_ns: glitch filter — pulses shorter than this are dropped.
    //   SBUS bits are 10us wide; PPM pulses 300+us. 1us is safe.
    // signal_range_max_ns: idle timeout — pulse longer than this completes the receive.
    //   Should be > frame interval. SBUS: 14ms. PPM: 20ms. Pick 12ms.
    rx_config.signal_range_min_ns = 1000;       // 1 µs (glitch filter)
    rx_config.signal_range_max_ns = idle_threshold_us * 1000;  // 3ms → completes after frame gap

    // ---- Start first receive ----
    armed_buf = buf_a;
    process_buf = buf_b;
    err = rmt_receive(rx_chan, armed_buf, sizeof(buf_a), &rx_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "initial rmt_receive failed: %s", esp_err_to_name(err));
        rmt_disable(rx_chan);
        vQueueDelete(recv_queue);
        recv_queue = nullptr;
        rmt_del_channel(rx_chan);
        rx_chan = nullptr;
        return;
    }

    initialised = true;
    enabled = true;
    ESP_LOGI(TAG, "RMT RX initialised on GPIO %d (resolution=%lu Hz, idle=%lu us)",
             HAL_ESP32_RCIN, (unsigned long)resolution_hz, (unsigned long)idle_threshold_us);
}

void RmtSigReader::disable()
{
    if (!initialised || !enabled) {
        return;
    }
    rmt_disable(rx_chan);
    enabled = false;
}

void RmtSigReader::rearm_rx()
{
    if (!initialised || !enabled || rx_chan == nullptr) {
        return;
    }
    esp_err_t err = rmt_receive(rx_chan, armed_buf, sizeof(buf_a), &rx_config);
    if (err != ESP_OK) {
        // Could happen transiently if RX is busy. Best effort — RC will recover next frame.
    }
}

// ISR — runs in interrupt context. Must be IRAM-safe.
bool IRAM_ATTR RmtSigReader::on_recv_done(rmt_channel_handle_t channel,
                                           const rmt_rx_done_event_data_t *edata,
                                           void *user_ctx)
{
    RmtSigReader *self = static_cast<RmtSigReader *>(user_ctx);
    BaseType_t high_task_wakeup = pdFALSE;
    if (self != nullptr && self->recv_queue != nullptr) {
        xQueueSendFromISR(self->recv_queue, edata, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

// Walk the symbol buffer and produce (high, low) pulse pairs.
// Returns true when a pair is ready; updates ready_high/ready_low fields.
// Same algorithm as legacy implementation — preserves AP_RCProtocol semantics.
bool RmtSigReader::add_item(uint32_t duration, bool level)
{
    bool has_more = true;
    if (duration == 0) {
        has_more = false;
        duration = idle_threshold_us;
    }
    if (level) {
        if (last_high == 0) {
            last_high = duration;
        }
    } else {
        if (last_high != 0) {
            ready_high = last_high;
            ready_low = duration;
            pulse_ready = true;
            last_high = 0;
        }
    }
    return has_more;
}

bool RmtSigReader::read(uint32_t &width_high, uint32_t &width_low)
{
    if (!initialised || !enabled) {
        return false;
    }

    // If we don't have a batch in progress, try to fetch one
    if (process_idx >= process_count) {
        rmt_rx_done_event_data_t edata;
        if (xQueueReceive(recv_queue, &edata, 0) != pdTRUE) {
            return false;  // no new data
        }
        // The buffer that just got filled IS armed_buf. Swap roles:
        //   process_buf <- just-filled buffer
        //   armed_buf   <- the OTHER buffer (re-armed below)
        rmt_symbol_word_t *just_filled = const_cast<rmt_symbol_word_t *>(edata.received_symbols);
        process_buf = just_filled;
        process_count = edata.num_symbols;
        process_idx = 0;
        armed_buf = (just_filled == buf_a) ? buf_b : buf_a;
        rearm_rx();
    }

    // Process ONE symbol per call — matches legacy driver semantics.
    // RCInput::_timer_tick() loops calling us until we return false, so all
    // pending symbols get processed within one timer tick. add_item() returns
    // false if duration==0 (end marker), in which case we should stop walking.
    rmt_symbol_word_t &sym = process_buf[process_idx];
    bool buffer_empty = !add_item(sym.duration0, sym.level0);
    if (!buffer_empty) {
        buffer_empty = !add_item(sym.duration1, sym.level1);
    }
    process_idx++;
    if (buffer_empty) {
        // End-marker symbol — skip remaining slots in this batch
        process_idx = process_count;
    }

    if (pulse_ready) {
        width_high = ready_high;
        width_low = ready_low;
        pulse_ready = false;
        return true;
    }
    return false;
}

#endif  // HAL_ESP32_RCIN
