/*
 * ESP32-S3 CAN Interface Implementation
 *
 * References:
 * - ArduRemoteID CANDriver.cpp (TWAI configuration)
 * - ChibiOS CANIface.cpp (HAL interface patterns)
 */

#include "CANIface.h"

#if HAL_NUM_CAN_IFACES

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/ExpandingString.h>
#include "Scheduler.h"

extern const AP_HAL::HAL& hal;

// Default TWAI GPIO pins (override in hwdef.dat)
#ifndef HAL_ESP32_CAN1_TX_PIN
#define HAL_ESP32_CAN1_TX_PIN GPIO_NUM_47
#endif

#ifndef HAL_ESP32_CAN1_RX_PIN
#define HAL_ESP32_CAN1_RX_PIN GPIO_NUM_38
#endif

// ESP32 clock configuration
// TWAI peripheral runs from 80 MHz APB clock (no divider applied)
// Using 40 MHz would require explicit clock source reconfiguration
#define ESP32_CAN_PERIPHERAL_CLK_HZ 80000000UL
#define ESP32_CAN_EFFECTIVE_CLK_HZ ESP32_CAN_PERIPHERAL_CLK_HZ  // Use real 80MHz, not 40MHz

namespace ESP32 {

// ==========================================================================
// Constructor and Initialization
// ==========================================================================

CANIface::CANIface(uint8_t index)
    : self_index_(index)
    , initialized_(false)
    , busoff_detected_(false)
    , rx_bytebuffer_(sizeof(CanRxItem) * HAL_CAN_RX_QUEUE_SIZE)
    , rx_queue_(&rx_bytebuffer_)
    , tx_queue_head_(0)
    , tx_queue_tail_(0)
    , tx_push_index_(0)
    , event_sem_(nullptr)
    , last_bus_recovery_us_(0)
    , last_error_poll_us_(0)
    , acceptance_code_(0)
    , acceptance_mask_(0xFFFFFFFF)
    , filters_configured_(false)
    , num_sw_filters_(0)
    , use_sw_filtering_(false)
    , tx_pin_(HAL_ESP32_CAN1_TX_PIN)
    , rx_pin_(HAL_ESP32_CAN1_RX_PIN)
{
#if !defined(HAL_BOOTLOADER_BUILD)
    memset(&stats_, 0, sizeof(stats_));
    memset(&extended_stats_, 0, sizeof(extended_stats_));
#endif
}

bool CANIface::init(const uint32_t bitrate, const OperatingMode mode)
{
    if (initialized_) {
        return true;
    }

    // Store configuration
    mode_ = mode;
    bitrate_ = bitrate;

    /*
     * ArduRemoteID Optimization Strategy:
     * Default to priority-based filtering to prevent CPU overload on busy CAN bus
     *
     * Filter configuration:
     * - acceptance_code = 0x10000000 << 3 (priority bit 4 set)
     * - acceptance_mask = 0x0FFFFFFF << 3 (check priority bits only)
     *
     * Result: Only accept messages with priority >= 16 (CANARD_TRANSFER_PRIORITY_MEDIUM or lower)
     * High-rate messages (ESC commands, etc.) with priority < 16 are hardware-filtered
     */
    if (!filters_configured_) {
        acceptance_code_ = 0x10000000U << 3;
        acceptance_mask_ = 0x0FFFFFFFU << 3;
    }

    // Initialize TWAI driver
    if (!initTWAI(bitrate, acceptance_code_, acceptance_mask_)) {
        hal.console->printf("CAN%d: TWAI init failed\\n", self_index_);
        return false;
    }

    // Setup TWAI alerts
    setupTWAIAlerts();

    // Start TWAI driver
    esp_err_t err = twai_start();
    if (err != ESP_OK) {
        hal.console->printf("CAN%d: TWAI start failed: %d\\n", self_index_, err);
        return false;
    }

    initialized_ = true;
    hal.console->printf("CAN%d: Initialized at %lu bps (mode=%d)\\n",
                       self_index_, bitrate, (int)mode);

    return true;
}

bool CANIface::initTWAI(uint32_t bitrate, uint32_t acceptance_code, uint32_t acceptance_mask)
{
    // Compute bit timings
    Timings timings;
    if (!computeTimings(bitrate, timings)) {
        hal.console->printf("CAN%d: Failed to compute timings for %lu bps\\n",
                           self_index_, bitrate);
        return false;
    }

    // TWAI timing configuration
    twai_timing_config_t t_config = {};

    // Use ESP-IDF predefined configurations when possible
    switch (bitrate) {
        case 1000000:
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
        case 500000:
            t_config = TWAI_TIMING_CONFIG_500KBITS();
            break;
        case 250000:
            t_config = TWAI_TIMING_CONFIG_250KBITS();
            break;
        case 125000:
            t_config = TWAI_TIMING_CONFIG_125KBITS();
            break;
        default:
            // Custom timing
            t_config.brp = timings.prescaler + 1;
            t_config.tseg_1 = timings.bs1 + 1;
            t_config.tseg_2 = timings.bs2 + 1;
            t_config.sjw = timings.sjw + 1;
            t_config.triple_sampling = false;
            break;
    }

    // TWAI filter configuration
    twai_filter_config_t f_config = {
        .acceptance_code = acceptance_code,
        .acceptance_mask = acceptance_mask,
        .single_filter = true
    };

    // TWAI general configuration (from ArduRemoteID proven settings)
    twai_mode_t twai_mode = TWAI_MODE_NORMAL;
    switch (mode_) {
        case NormalMode:
            twai_mode = TWAI_MODE_NORMAL;
            break;
        case SilentMode:
            twai_mode = TWAI_MODE_LISTEN_ONLY;
            break;
        default:
            twai_mode = TWAI_MODE_NORMAL;
            break;
    }

    twai_general_config_t g_config = {
        .mode = twai_mode,
        .tx_io = tx_pin_,
        .rx_io = rx_pin_,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 5,              // ArduRemoteID setting
        .rx_queue_len = 50,             // ArduRemoteID setting (CRITICAL!)
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0,
        .intr_flags = ESP_INTR_FLAG_LEVEL2
    };

    // Install TWAI driver
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        hal.console->printf("CAN%d: twai_driver_install failed: %d\\n", self_index_, err);
        return false;
    }

    hal.console->printf("CAN%d: TWAI driver installed (TX=GPIO%d, RX=GPIO%d)\\n",
                       self_index_, tx_pin_, rx_pin_);
    return true;
}

void CANIface::setupTWAIAlerts()
{
    // ArduRemoteID strategy: Enable necessary alerts + TX alerts for semaphore signaling
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA |
                                TWAI_ALERT_RX_QUEUE_FULL |
                                TWAI_ALERT_TX_IDLE |
                                TWAI_ALERT_TX_SUCCESS |
                                TWAI_ALERT_BUS_ERROR |
                                TWAI_ALERT_ERR_PASS |
                                TWAI_ALERT_BUS_OFF;

    esp_err_t err = twai_reconfigure_alerts(alerts_to_enable, NULL);
    if (err == ESP_OK) {
        hal.console->printf("CAN%d: TWAI alerts configured\\n", self_index_);
    } else {
        hal.console->printf("CAN%d: Failed to configure alerts: %d\\n", self_index_, err);
    }
}

// Poll TWAI alerts and signal event semaphore when data is available
void CANIface::pollAlerts()
{
    if (!initialized_) {
        return;
    }

    uint32_t alerts_triggered = 0;
    esp_err_t err = twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(0));

    if (err == ESP_OK && alerts_triggered != 0) {
        bool should_signal = false;

        // Check for RX events (data available for reading)
        if (alerts_triggered & (TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL)) {
            should_signal = true;
        }

        // Check for TX events (queue has space or transmission complete)
        if (alerts_triggered & (TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS)) {
            should_signal = true;
        }

        // Signal the event semaphore to wake up waiting threads
        if (should_signal && event_sem_ != nullptr) {
            event_sem_->signal();
        }

        // Handle error alerts
        if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
            busoff_detected_ = true;
            hal.console->printf("CAN%d: Bus-off detected\\n", self_index_);
        }

        if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
            hal.console->printf("CAN%d: Error passive threshold exceeded\\n", self_index_);
        }

        if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
#if !defined(HAL_BOOTLOADER_BUILD)
            extended_stats_.rx_hw_errors++;
#endif
        }
    }
}

// ==========================================================================
// Bit Timing Computation (from ArduRemoteID)
// ==========================================================================

bool CANIface::computeTimings(uint32_t target_bitrate, Timings& out_timings)
{
    if (target_bitrate < 1) {
        return false;
    }

    const uint32_t pclk = ESP32_CAN_EFFECTIVE_CLK_HZ;

    static const int MaxBS1 = 16;
    static const int MaxBS2 = 8;

    /*
     * Optimal quanta per bit (from ArduRemoteID):
     *   Bitrate        Optimal Maximum
     *   1000 kbps      8       10
     *   500  kbps      16      17
     *   250  kbps      16      17
     *   125  kbps      16      17
     */
    const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;
    static const int MaxSamplePointLocation = 900;  // 90%

    // BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))
    const uint32_t prescaler_bs = pclk / target_bitrate;

    // Search for highest quanta per bit
    uint8_t bs1_bs2_sum = uint8_t(max_quanta_per_bit - 1);

    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
        if (bs1_bs2_sum <= 2) {
            return false;  // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U)) {
        return false;
    }

    // Compute BS1 and BS2 for optimal sample point (87.5%)
    struct BsPair {
        uint8_t bs1;
        uint8_t bs2;
        uint16_t sample_point_permill;

        BsPair(uint8_t bs1_bs2_sum, uint8_t arg_bs1)
            : bs1(arg_bs1)
            , bs2(uint8_t(bs1_bs2_sum - bs1))
            , sample_point_permill(uint16_t(1000 * (1 + bs1) / (1 + bs1 + bs2)))
        {}

        bool isValid() const {
            return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);
        }
    };

    // First attempt with rounding to nearest
    BsPair solution(bs1_bs2_sum, uint8_t(((7 * bs1_bs2_sum - 1) + 4) / 8));

    if (solution.sample_point_permill > MaxSamplePointLocation) {
        // Second attempt with rounding to zero
        solution = BsPair(bs1_bs2_sum, uint8_t((7 * bs1_bs2_sum - 1) / 8));
    }

    if ((target_bitrate != (pclk / (prescaler * (1 + solution.bs1 + solution.bs2)))) ||
        !solution.isValid()) {
        return false;
    }

    hal.console->printf("CAN%d: Timings - quanta/bit: %d, sample point: %.1f%%\\n",
                       self_index_,
                       int(1 + solution.bs1 + solution.bs2),
                       float(solution.sample_point_permill) / 10.0f);

    out_timings.prescaler = uint16_t(prescaler - 1U);
    out_timings.sjw = 0;  // Which means 1
    out_timings.bs1 = uint8_t(solution.bs1 - 1);
    out_timings.bs2 = uint8_t(solution.bs2 - 1);

    return true;
}

// ==========================================================================
// Send and Receive (ArduRemoteID pattern)
// ==========================================================================

int16_t CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                       CanIOFlags flags)
{
    if (!initialized_) {
        return -1;
    }

#if !defined(HAL_BOOTLOADER_BUILD)
    stats_.tx_requests++;
#endif

    // Check queue space
    uint8_t next_tail = (tx_queue_tail_ + 1) % HAL_CAN_TX_QUEUE_SIZE;
    if (next_tail == tx_queue_head_) {
#if !defined(HAL_BOOTLOADER_BUILD)
        stats_.tx_overflow++;
#endif
        return 0;  // Queue full
    }

    // Add to priority queue
    TxItem& item = tx_queue_[tx_queue_tail_];
    item.item.frame = frame;
    item.item.deadline = tx_deadline;
    item.item.loopback = (flags & Loopback) != 0;
    item.item.abort_on_error = (flags & AbortOnError) != 0;
    item.item.pushed = true;
    item.item.setup = true;
    item.push_index = tx_push_index_++;

    tx_queue_tail_ = next_tail;

    // Try immediate send
    processTxQueue();

    return 1;
}

void CANIface::processTxQueue()
{
    while (tx_queue_head_ != tx_queue_tail_) {
        TxItem& item = tx_queue_[tx_queue_head_];

        // Convert to TWAI message
        twai_message_t message;
        convertFrameToTWAI(item.item.frame, message);

        // Check bus state and auto-recover (ArduRemoteID pattern)
        twai_status_info_t status_info;
        esp_err_t err = twai_get_status_info(&status_info);

        if (err == ESP_OK) {
            switch (status_info.state) {
                case TWAI_STATE_STOPPED:
                    twai_start();
                    break;

                case TWAI_STATE_BUS_OFF: {
                    busoff_detected_ = true;
                    uint64_t now_us = AP_HAL::micros64();
                    if (now_us - last_bus_recovery_us_ > 2000000) {  // 2 seconds
                        last_bus_recovery_us_ = now_us;
                        twai_initiate_recovery();
                        hal.console->printf("CAN%d: Initiating bus-off recovery\\n", self_index_);
                    }
                    return;  // Don't try to send during recovery
                }

                case TWAI_STATE_RECOVERING:
                    return;  // Wait for recovery

                default:
                    busoff_detected_ = false;
                    break;
            }
        }

        // Attempt transmission (5ms timeout from ArduRemoteID)
        err = twai_transmit(&message, pdMS_TO_TICKS(5));

        if (err == ESP_OK) {
            // Success
#if !defined(HAL_BOOTLOADER_BUILD)
            stats_.tx_success++;
            stats_.last_transmit_us = AP_HAL::micros64();
#endif
            tx_queue_head_ = (tx_queue_head_ + 1) % HAL_CAN_TX_QUEUE_SIZE;
            last_bus_recovery_us_ = 0;  // Clear recovery timer
        } else if (err == ESP_ERR_TIMEOUT) {
            // No space in TX queue, try again later
            break;
        } else {
            // Other error, discard frame
#if !defined(HAL_BOOTLOADER_BUILD)
            stats_.tx_rejected++;
#endif
            tx_queue_head_ = (tx_queue_head_ + 1) % HAL_CAN_TX_QUEUE_SIZE;
        }
    }
}

int16_t CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us,
                          CanIOFlags& out_flags)
{
    if (!initialized_) {
        return -1;
    }

    // First try to get from our queue
    CanRxItem rx_item;
    if (rx_queue_.pop(rx_item)) {
        out_frame = rx_item.frame;
        out_timestamp_us = rx_item.timestamp_us;
        out_flags = rx_item.flags;
        return 1;
    }

    // Then try to receive from TWAI (ArduRemoteID: batch receive up to 60 frames)
    twai_message_t message;
    uint8_t count = 60;

    while (count-- > 0) {
        esp_err_t err = twai_receive(&message, pdMS_TO_TICKS(0));  // Non-blocking

        if (err == ESP_OK) {
            // Convert and add to queue
            CanRxItem item;
            convertTWAIToFrame(message, item.frame);

            // Apply software filter if enabled
            if (use_sw_filtering_) {
                uint32_t can_id = item.frame.id;
                if (!passesSoftwareFilters(can_id)) {
                    // Message filtered out by software filter
#if !defined(HAL_BOOTLOADER_BUILD)
                    stats_.rx_errors++;  // Count as filtered
#endif
                    continue;  // Skip this message
                }
            }

            item.timestamp_us = AP_HAL::micros64();
            item.flags = 0;

#if !defined(HAL_BOOTLOADER_BUILD)
            stats_.rx_received++;
#endif

            if (!rx_queue_.push(item)) {
#if !defined(HAL_BOOTLOADER_BUILD)
                stats_.rx_overflow++;
#endif
                break;
            }
        } else {
            break;  // No more frames
        }
    }

    // Try to pop from queue again
    if (rx_queue_.pop(rx_item)) {
        out_frame = rx_item.frame;
        out_timestamp_us = rx_item.timestamp_us;
        out_flags = rx_item.flags;
        return 1;
    }

    return 0;  // No frame available
}

// ==========================================================================
// Frame Conversion Helpers
// ==========================================================================

void CANIface::convertFrameToTWAI(const AP_HAL::CANFrame &frame, twai_message_t &message)
{
    memset(&message, 0, sizeof(message));

    message.identifier = frame.id & AP_HAL::CANFrame::MaskExtID;
    message.extd = frame.isExtended() ? 1 : 0;
    message.rtr = frame.isRemoteTransmissionRequest() ? 1 : 0;
    message.data_length_code = frame.dlc;

    memcpy(message.data, frame.data, MIN(frame.dlc, 8));
}

void CANIface::convertTWAIToFrame(const twai_message_t &message, AP_HAL::CANFrame &frame)
{
    frame.id = message.identifier;

    if (message.extd) {
        frame.id |= AP_HAL::CANFrame::FlagEFF;
    }
    if (message.rtr) {
        frame.id |= AP_HAL::CANFrame::FlagRTR;
    }

    frame.dlc = message.data_length_code;
    memcpy(frame.data, message.data, MIN(message.data_length_code, 8));
}

// ==========================================================================
// Filter Configuration
// ==========================================================================

/*
  Optimize multiple filters by finding the best common mask
  Returns true if filters can be merged efficiently
*/
bool CANIface::optimizeFilterMerge(const CanFilterConfig* configs, uint16_t num_configs,
                                   uint32_t& out_code, uint32_t& out_mask)
{
    if (num_configs == 0) {
        return false;
    }

    // Find common bits across all filter IDs
    uint32_t common_bits = 0xFFFFFFFF;
    uint32_t differing_bits = 0;

    for (uint16_t i = 0; i < num_configs; i++) {
        uint32_t id = configs[i].id;
        if (i == 0) {
            out_code = id;
        } else {
            // Find bits that differ from first ID
            uint32_t diff = out_code ^ id;
            differing_bits |= diff;
            common_bits &= ~diff;
        }
    }

    // Create mask that only checks common bits
    out_mask = common_bits;
    out_code &= common_bits;

    // Calculate efficiency: how many unwanted messages will pass?
    // If more than 50% of the address space passes, it's inefficient
    uint32_t dont_care_bits = __builtin_popcount(~common_bits & 0x1FFFFFFF);
    uint32_t unwanted_pass_count = (1U << dont_care_bits);

    // If too many unwanted messages would pass, use software filtering
    return (unwanted_pass_count < 1024);  // Accept if <1024 extra messages
}

/*
  Check if a CAN ID passes the software filters
  Enhanced with per-filter hit counting and adaptive reordering
*/
bool CANIface::passesSoftwareFilters(uint32_t can_id) const
{
    if (!use_sw_filtering_ || num_sw_filters_ == 0) {
        return true;  // No software filtering, accept all
    }

    // OPTIMIZATION: Check against all software filters
    // Most commonly hit filters are checked first (updated periodically)
    for (uint8_t i = 0; i < num_sw_filters_; i++) {
        if (!sw_filters_[i].active) {
            continue;
        }

        uint32_t masked_id = can_id & sw_filters_[i].mask;
        uint32_t filter_id = sw_filters_[i].id & sw_filters_[i].mask;

        if (masked_id == filter_id) {
#if !defined(HAL_BOOTLOADER_BUILD)
            // Track which filter matched (const_cast for statistics in const function)
            const_cast<CANIface*>(this)->extended_stats_.sw_filter_hits[i]++;

            // OPTIMIZATION: If this filter is hot and not first, consider reordering
            // Every 1000 hits, check if we should swap with previous filter
            if (i > 0 && (extended_stats_.sw_filter_hits[i] % 1000 == 0)) {
                if (extended_stats_.sw_filter_hits[i] > extended_stats_.sw_filter_hits[i-1] * 2) {
                    // This filter is hit 2x more than previous, swap them
                    const_cast<CANIface*>(this)->reorderSoftwareFilters(i);
                }
            }
#endif
            return true;  // Matches this filter
        }
    }

#if !defined(HAL_BOOTLOADER_BUILD)
    // Track software filter rejections
    const_cast<CANIface*>(this)->extended_stats_.rx_sw_filtered++;
#endif

    return false;  // No filter matched
}

/*
  Adaptive filter reordering - move hot filters towards the front
  This is called when a filter becomes significantly hotter than its predecessor
*/
void CANIface::reorderSoftwareFilters(uint8_t hot_index)
{
    if (hot_index == 0 || hot_index >= num_sw_filters_) {
        return;  // Can't reorder first filter or invalid index
    }

    // Swap with previous filter
    SoftwareFilter temp = sw_filters_[hot_index];
    sw_filters_[hot_index] = sw_filters_[hot_index - 1];
    sw_filters_[hot_index - 1] = temp;

#if !defined(HAL_BOOTLOADER_BUILD)
    // Swap hit counts too
    uint32_t temp_hits = extended_stats_.sw_filter_hits[hot_index];
    extended_stats_.sw_filter_hits[hot_index] = extended_stats_.sw_filter_hits[hot_index - 1];
    extended_stats_.sw_filter_hits[hot_index - 1] = temp_hits;
#endif
}

bool CANIface::configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs)
{
    /*
     * ESP32 TWAI Enhanced Filtering Strategy:
     *
     * 1. Single hardware filter + software filter layer
     * 2. Optimize hardware filter to catch common patterns
     * 3. Use software filters for precise matching
     * 4. Dramatically reduces CPU load on busy CAN bus
     */

    // Calculate new filter settings
    uint32_t new_acceptance_code;
    uint32_t new_acceptance_mask;
    bool new_use_sw_filtering;
    uint8_t new_num_sw_filters;
    SoftwareFilter new_sw_filters[MAX_SW_FILTERS];

    if (num_configs == 0) {
        // Accept all
        new_acceptance_code = 0;
        new_acceptance_mask = 0xFFFFFFFF;
        new_use_sw_filtering = false;
        new_num_sw_filters = 0;
    } else if (num_configs == 1) {
        // Single filter: use hardware only
        new_acceptance_code = filter_configs[0].id << 3;
        new_acceptance_mask = filter_configs[0].mask << 3;
        new_use_sw_filtering = false;
        new_num_sw_filters = 0;

        hal.console->printf("CAN%d: Hardware filter: ID=0x%08lx Mask=0x%08lx\n",
                           self_index_, new_acceptance_code, new_acceptance_mask);
    } else {
        // Multiple filters: try to merge efficiently
        uint32_t hw_code, hw_mask;
        bool can_merge = optimizeFilterMerge(filter_configs, num_configs, hw_code, hw_mask);

        if (can_merge) {
            // Hardware filter catches most, software filters refine
            new_acceptance_code = hw_code << 3;
            new_acceptance_mask = hw_mask << 3;
            new_use_sw_filtering = true;

            // Store all filters in software filter table
            new_num_sw_filters = MIN(num_configs, MAX_SW_FILTERS);
            for (uint8_t i = 0; i < new_num_sw_filters; i++) {
                new_sw_filters[i].id = filter_configs[i].id;
                new_sw_filters[i].mask = filter_configs[i].mask;
                new_sw_filters[i].active = true;
            }

            hal.console->printf("CAN%d: Hybrid filtering: HW(0x%08lx/0x%08lx) + SW(%u filters)\n",
                               self_index_, new_acceptance_code, new_acceptance_mask, new_num_sw_filters);
        } else {
            // Cannot merge efficiently, use priority filtering
            hal.console->printf("CAN%d: Using priority filter (filters too diverse)\n",
                               self_index_);
            new_acceptance_code = 0x10000000U << 3;  // Priority >= 16
            new_acceptance_mask = 0x0FFFFFFFU << 3;
            new_use_sw_filtering = false;
            new_num_sw_filters = 0;
        }
    }

    // If already initialized, need to stop/reinstall/restart TWAI to apply new filters
    if (initialized_) {
        hal.console->printf("CAN%d: Dynamic filter reconfiguration (stop->reinstall->start)\n",
                           self_index_);

        // Step 1: Stop TWAI driver
        esp_err_t err = twai_stop();
        if (err != ESP_OK) {
            hal.console->printf("CAN%d: Failed to stop TWAI: %d\n", self_index_, err);
            return false;
        }

        // Step 2: Uninstall TWAI driver
        err = twai_driver_uninstall();
        if (err != ESP_OK) {
            hal.console->printf("CAN%d: Failed to uninstall TWAI: %d\n", self_index_, err);
            return false;
        }

        // Step 3: Update filter settings
        acceptance_code_ = new_acceptance_code;
        acceptance_mask_ = new_acceptance_mask;
        use_sw_filtering_ = new_use_sw_filtering;
        num_sw_filters_ = new_num_sw_filters;
        if (use_sw_filtering_) {
            memcpy(sw_filters_, new_sw_filters, sizeof(SoftwareFilter) * num_sw_filters_);
        }

        // Step 4: Reinstall TWAI driver with new filters
        if (!initTWAI(bitrate_, acceptance_code_, acceptance_mask_)) {
            hal.console->printf("CAN%d: Failed to reinstall TWAI with new filters\n", self_index_);
            initialized_ = false;
            return false;
        }

        // Step 5: Reapply alerts configuration
        setupTWAIAlerts();

        // Step 6: Restart TWAI driver
        err = twai_start();
        if (err != ESP_OK) {
            hal.console->printf("CAN%d: Failed to restart TWAI: %d\n", self_index_, err);
            initialized_ = false;
            return false;
        }

        hal.console->printf("CAN%d: Filter reconfiguration complete\n", self_index_);
    } else {
        // Not initialized yet, just store filter settings
        acceptance_code_ = new_acceptance_code;
        acceptance_mask_ = new_acceptance_mask;
        use_sw_filtering_ = new_use_sw_filtering;
        num_sw_filters_ = new_num_sw_filters;
        if (use_sw_filtering_) {
            memcpy(sw_filters_, new_sw_filters, sizeof(SoftwareFilter) * num_sw_filters_);
        }
    }

    filters_configured_ = true;
    return true;
}

// ==========================================================================
// Event Handling and Select
// ==========================================================================

bool CANIface::select(bool &read_select, bool &write_select,
                      const AP_HAL::CANFrame* const pending_tx,
                      uint64_t timeout_us)
{
    if (!initialized_) {
        return false;
    }

    uint64_t start_us = AP_HAL::micros64();

    do {
        // Poll TWAI alerts for event-driven wake-up
        pollAlerts();

        // Check read availability
        if (read_select) {
            read_select = !rx_queue_.is_empty();

            // If still false, try to receive from TWAI
            if (!read_select) {
                twai_message_t message;
                if (twai_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK) {
                    CanRxItem item;
                    convertTWAIToFrame(message, item.frame);

                    // Apply software filter if enabled
                    if (use_sw_filtering_) {
                        uint32_t can_id = item.frame.id;
                        if (!passesSoftwareFilters(can_id)) {
                            // Message filtered out by software filter
#if !defined(HAL_BOOTLOADER_BUILD)
                            extended_stats_.rx_sw_filtered++;
                            stats_.rx_errors++;  // Keep backward compatibility
#endif
                            continue;  // Skip this message
                        }
                    }

                    item.timestamp_us = AP_HAL::micros64();
                    item.flags = 0;

                    if (rx_queue_.push(item)) {
                        read_select = true;
#if !defined(HAL_BOOTLOADER_BUILD)
                        stats_.rx_received++;
#endif
                    }
                }
            }
        }

        // Check write availability
        if (write_select) {
            uint8_t next_tail = (tx_queue_tail_ + 1) % HAL_CAN_TX_QUEUE_SIZE;
            write_select = (next_tail != tx_queue_head_);
        }

        // If both conditions met or timeout
        if ((read_select || !pending_tx) && write_select) {
            return true;
        }

        // Check timeout - timeout_us is an absolute deadline, not a relative duration
        uint64_t now = AP_HAL::micros64();
        if (now >= timeout_us) {
            break;  // Deadline reached
        }

        // Wait a bit
        if (event_sem_ != nullptr) {
            uint64_t remaining_us = timeout_us - now;

            if (remaining_us > 0) {
                // BinarySemaphore::wait() expects microseconds (it converts to ms internally)
                event_sem_->wait(remaining_us);
            }
        } else {
            hal.scheduler->delay_microseconds(100);
        }

    } while (true);  // Loop controlled by timeout check above

    return false;
}

bool CANIface::set_event_handle(AP_HAL::BinarySemaphore *sem_handle)
{
    event_sem_ = sem_handle;
    return true;
}

bool CANIface::add_to_rx_queue(const CanRxItem &rx_item)
{
    return rx_queue_.push(rx_item);
}

// ==========================================================================
// Status and Statistics
// ==========================================================================

uint32_t CANIface::getErrorCount() const
{
#if !defined(HAL_BOOTLOADER_BUILD)
    return stats_.rx_errors + stats_.num_busoff_err;
#else
    return 0;
#endif
}

bool CANIface::is_busoff() const
{
    if (!initialized_) {
        return false;
    }

    twai_status_info_t status_info;
    if (twai_get_status_info(&status_info) == ESP_OK) {
        return status_info.state == TWAI_STATE_BUS_OFF;
    }

    return busoff_detected_;
}

#if !defined(HAL_BOOTLOADER_BUILD)
void CANIface::get_stats(ExpandingString &str)
{
    str.printf("CAN%d: TX req:%lu suc:%lu rej:%lu ovf:%lu timeout:%lu\\n"
               "      RX rcv:%lu ovf:%lu err:%lu\\n"
               "      Bus-off:%lu Last TX:%llu us\\n",
               self_index_,
               stats_.tx_requests,
               stats_.tx_success,
               stats_.tx_rejected,
               stats_.tx_overflow,
               stats_.tx_timedout,
               stats_.rx_received,
               stats_.rx_overflow,
               stats_.rx_errors,
               stats_.num_busoff_err,
               stats_.last_transmit_us);

    // Add extended statistics if software filtering is enabled
    if (use_sw_filtering_) {
        str.printf("      Filter: SW-filtered:%lu HW-err:%lu\\n",
                   extended_stats_.rx_sw_filtered,
                   extended_stats_.rx_hw_errors);

        // Show per-filter hit counts
        if (num_sw_filters_ > 0) {
            str.printf("      SW Filter hits:");
            for (uint8_t i = 0; i < num_sw_filters_; i++) {
                if (sw_filters_[i].active) {
                    str.printf(" [%u]=%lu", i, extended_stats_.sw_filter_hits[i]);
                }
            }
            str.printf("\\n");
        }
    }
}
#endif

// ==========================================================================
// Test and Debug Methods
// ==========================================================================

void CANIface::flush_tx()
{
    tx_queue_head_ = 0;
    tx_queue_tail_ = 0;
    tx_push_index_ = 0;
}

void CANIface::clear_rx()
{
    rx_queue_.clear();

    // Drain TWAI RX queue
    twai_message_t message;
    while (twai_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK) {
        // Discard
    }
}

}  // namespace ESP32

#endif  // HAL_NUM_CAN_IFACES
