/**
 * @file lora_mavlink.h
 * @brief LoRa MAVLink communication interface for ESP32-S3
 *
 * Provides LoRa-based MAVLink telemetry using SX1262 radio module.
 * Operates as a virtual UART (SERIAL4) for ArduPilot integration.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// LoRa configuration from hwdef.dat
#ifndef LORA_PIN_SCK
#define LORA_PIN_SCK     35
#endif
#ifndef LORA_PIN_MISO
#define LORA_PIN_MISO    36
#endif
#ifndef LORA_PIN_MOSI
#define LORA_PIN_MOSI    37
#endif
#ifndef LORA_PIN_CS
#define LORA_PIN_CS      39
#endif
#ifndef LORA_PIN_RST
#define LORA_PIN_RST     42
#endif
#ifndef LORA_PIN_BUSY
#define LORA_PIN_BUSY    40
#endif
#ifndef LORA_PIN_DIO1
#define LORA_PIN_DIO1    41
#endif

#ifndef LORA_FREQ_HZ
#define LORA_FREQ_HZ     433000000
#endif
#ifndef LORA_TX_POWER
#define LORA_TX_POWER    20
#endif
#ifndef LORA_SF
#define LORA_SF          8
#endif
#ifndef LORA_BW
#define LORA_BW          125
#endif

// Buffer sizes
#define LORA_RX_BUFFER_SIZE     512
#define LORA_TX_BUFFER_SIZE     512
#define LORA_MAX_PACKET_SIZE    255

// LoRa state
typedef enum {
    LORA_STATE_IDLE,
    LORA_STATE_RX,
    LORA_STATE_TX,
    LORA_STATE_CAD,
    LORA_STATE_ERROR
} lora_state_t;

// LoRa configuration structure
typedef struct {
    uint32_t frequency;     // Frequency in Hz
    int8_t tx_power;        // TX power in dBm
    uint8_t spreading_factor;
    uint8_t bandwidth;      // 0=125kHz, 1=250kHz, 2=500kHz
    uint8_t coding_rate;    // 1=4/5, 2=4/6, 3=4/7, 4=4/8
    uint16_t preamble_len;
    bool crc_enable;
} lora_config_t;

// LoRa statistics
typedef struct {
    uint32_t tx_packets;
    uint32_t rx_packets;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t crc_errors;
    int16_t last_rssi;
    int8_t last_snr;
} lora_stats_t;

/**
 * @brief Initialize LoRa MAVLink interface
 * @param config LoRa configuration (NULL for defaults)
 * @return true on success
 */
bool lora_mavlink_init(const lora_config_t *config);

/**
 * @brief Deinitialize LoRa interface
 */
void lora_mavlink_deinit(void);

/**
 * @brief Write data to LoRa TX buffer
 * @param data Data to send
 * @param len Length of data
 * @return Number of bytes written
 */
size_t lora_mavlink_write(const uint8_t *data, size_t len);

/**
 * @brief Read data from LoRa RX buffer
 * @param data Buffer to store received data
 * @param len Maximum bytes to read
 * @return Number of bytes read
 */
size_t lora_mavlink_read(uint8_t *data, size_t len);

/**
 * @brief Get number of bytes available in RX buffer
 * @return Number of bytes available
 */
size_t lora_mavlink_available(void);

/**
 * @brief Get TX buffer free space
 * @return Number of bytes free in TX buffer
 */
size_t lora_mavlink_tx_free(void);

/**
 * @brief Process LoRa state machine (call periodically)
 */
void lora_mavlink_process(void);

/**
 * @brief Get current LoRa state
 * @return Current state
 */
lora_state_t lora_mavlink_get_state(void);

/**
 * @brief Get LoRa statistics
 * @param stats Pointer to stats structure
 */
void lora_mavlink_get_stats(lora_stats_t *stats);

/**
 * @brief Set LoRa frequency
 * @param freq_hz Frequency in Hz
 * @return true on success
 */
bool lora_mavlink_set_frequency(uint32_t freq_hz);

/**
 * @brief Set TX power
 * @param power_dbm Power in dBm (2-22)
 * @return true on success
 */
bool lora_mavlink_set_tx_power(int8_t power_dbm);

/**
 * @brief Check if LoRa is initialized
 * @return true if initialized
 */
bool lora_mavlink_is_initialized(void);

#ifdef __cplusplus
}
#endif
