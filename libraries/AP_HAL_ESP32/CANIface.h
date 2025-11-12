/*
 * ESP32-S3 CAN Interface Implementation
 * Based on ArduRemoteID CANDriver and ChibiOS::CANIface
 *
 * This implementation combines:
 * - ArduRemoteID's proven TWAI driver configuration
 * - ArduPilot HAL's standard CANIface interface
 * - Optimizations for ESP32's limited CAN hardware
 */

#pragma once

#include "AP_HAL_ESP32.h"

#if HAL_NUM_CAN_IFACES

#include <driver/twai.h>
#include <esp_err.h>

#ifndef HAL_CAN_RX_QUEUE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

#ifndef HAL_CAN_TX_QUEUE_SIZE
#define HAL_CAN_TX_QUEUE_SIZE 32
#endif

static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");

namespace ESP32 {

class CANIface : public AP_HAL::CANIface
{
public:
    CANIface(uint8_t index);

    // ========== HAL Interface Implementation ==========

    // Initialize CAN interface
    bool init(const uint32_t bitrate, const OperatingMode mode) override;

    // Send CAN frame (returns: <0=error, 0=no_space, 1=success)
    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                 CanIOFlags flags) override;

    // Receive CAN frame (returns: <0=error, 0=no_frame, 1=success)
    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us,
                    CanIOFlags& out_flags) override;

    // Configure hardware filters
    bool configureFilters(const CanFilterConfig* filter_configs,
                          uint16_t num_configs) override;

    // Wait for read/write events
    bool select(bool &read_select, bool &write_select,
                const AP_HAL::CANFrame* const pending_tx,
                uint64_t timeout_us) override;

    // Set event semaphore for ISR notification
    bool set_event_handle(AP_HAL::BinarySemaphore *sem_handle) override;

    // Get number of available filters (ESP32 limitation: only 1 effective filter)
    uint16_t getNumFilters() const override { return 1; }

    // Get error count
    uint32_t getErrorCount() const override;

    // Check if bus-off
    bool is_busoff() const override;

    // Check if initialized
    bool is_initialized() const override { return initialized_; }

#if !defined(HAL_BOOTLOADER_BUILD)
    // Get statistics
    void get_stats(ExpandingString &str) override;
    const bus_stats_t *get_statistics(void) const override { return &stats_; }
#endif

    // Test methods
    void flush_tx() override;
    void clear_rx() override;

protected:
    int8_t get_iface_num() const override { return self_index_; }
    bool add_to_rx_queue(const CanRxItem &rx_item) override;

private:
    // ========== TWAI Driver Management ==========

    struct Timings {
        uint16_t prescaler;
        uint8_t sjw;
        uint8_t bs1;
        uint8_t bs2;
    };

    bool computeTimings(uint32_t target_bitrate, Timings& out_timings);
    bool initTWAI(uint32_t bitrate, uint32_t acceptance_code, uint32_t acceptance_mask);
    void setupTWAIAlerts();
    void pollAlerts();  // Poll TWAI alerts and signal event semaphore

    // ========== Rx/Tx Processing ==========

    void processTxQueue();
    void processRxQueue();
    bool recoverFromBusOff();
    void pollErrorFlags();

    // ========== Hardware Access ==========

    bool readFromTWAI(twai_message_t &message);
    bool writeToTWAI(const twai_message_t &message);
    void convertFrameToTWAI(const AP_HAL::CANFrame &frame, twai_message_t &message);
    void convertTWAIToFrame(const twai_message_t &message, AP_HAL::CANFrame &frame);

    // ========== Member Variables ==========

    const uint8_t self_index_;

    // State flags
    bool initialized_ : 1;
    bool busoff_detected_ : 1;

    // Configuration (stored for dynamic reconfiguration)
    uint32_t bitrate_;
    OperatingMode mode_;

    // Rx buffer (uses ByteBuffer + ObjectBuffer)
    ByteBuffer rx_bytebuffer_;
    ObjectBuffer<CanRxItem> rx_queue_;

    // Tx priority queue
    struct TxItem {
        CanTxItem item;
        uint32_t push_index;  // For FIFO ordering within same priority

        bool operator<(const TxItem& rhs) const {
            // Higher priority frames first, then FIFO
            if (item.frame.priorityLowerThan(rhs.item.frame)) {
                return true;
            }
            if (item.frame.priorityHigherThan(rhs.item.frame)) {
                return false;
            }
            return push_index > rhs.push_index;
        }
    };
    TxItem tx_queue_[HAL_CAN_TX_QUEUE_SIZE];
    uint8_t tx_queue_head_;
    uint8_t tx_queue_tail_;
    uint32_t tx_push_index_;

    // Event notification
    AP_HAL::BinarySemaphore *event_sem_;

    // Timing
    uint64_t last_bus_recovery_us_;
    uint64_t last_error_poll_us_;

    // Software filter layer (for multi-filter emulation)
    static constexpr uint8_t MAX_SW_FILTERS = 16;

#if !defined(HAL_BOOTLOADER_BUILD)
    // Statistics
    bus_stats_t stats_;

    // Enhanced statistics (ESP32-specific)
    struct {
        uint32_t rx_hw_filtered;      // Messages filtered by hardware
        uint32_t rx_sw_filtered;      // Messages filtered by software
        uint32_t rx_hw_errors;        // Hardware reception errors
        uint32_t sw_filter_hits[MAX_SW_FILTERS];  // Hit count per software filter
    } extended_stats_;
#endif

    // Filter configuration
    uint32_t acceptance_code_;
    uint32_t acceptance_mask_;
    bool filters_configured_;

    // Software filter layer (for multi-filter emulation)
    struct SoftwareFilter {
        uint32_t id;
        uint32_t mask;
        bool active;
    };
    SoftwareFilter sw_filters_[MAX_SW_FILTERS];
    uint8_t num_sw_filters_;
    bool use_sw_filtering_;

    // Helper methods
    bool passesSoftwareFilters(uint32_t can_id) const;
    bool optimizeFilterMerge(const CanFilterConfig* configs, uint16_t num_configs,
                             uint32_t& out_code, uint32_t& out_mask);
    void reorderSoftwareFilters(uint8_t hot_index);  // Adaptive filter reordering

    // TWAI hardware pins (from hwdef)
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
};

}  // namespace ESP32

#endif  // HAL_NUM_CAN_IFACES
