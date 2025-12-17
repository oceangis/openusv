/**
 * @file sx126x_hal.h
 * @brief SX126x HAL interface for ESP32-S3
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// SX126x commands
#define SX126X_CMD_SET_SLEEP              0x84
#define SX126X_CMD_SET_STANDBY            0x80
#define SX126X_CMD_SET_FS                 0xC1
#define SX126X_CMD_SET_TX                 0x83
#define SX126X_CMD_SET_RX                 0x82
#define SX126X_CMD_STOP_TIMER_ON_PREAMBLE 0x9F
#define SX126X_CMD_SET_RX_DUTY_CYCLE      0x94
#define SX126X_CMD_SET_CAD                0xC5
#define SX126X_CMD_SET_TX_CONTINUOUS_WAVE 0xD1
#define SX126X_CMD_SET_TX_INFINITE_PREAMBLE 0xD2
#define SX126X_CMD_SET_REGULATOR_MODE     0x96
#define SX126X_CMD_CALIBRATE              0x89
#define SX126X_CMD_CALIBRATE_IMAGE        0x98
#define SX126X_CMD_SET_PA_CONFIG          0x95
#define SX126X_CMD_SET_RX_TX_FALLBACK_MODE 0x93

#define SX126X_CMD_WRITE_REGISTER         0x0D
#define SX126X_CMD_READ_REGISTER          0x1D
#define SX126X_CMD_WRITE_BUFFER           0x0E
#define SX126X_CMD_READ_BUFFER            0x1E

#define SX126X_CMD_SET_DIO_IRQ_PARAMS     0x08
#define SX126X_CMD_GET_IRQ_STATUS         0x12
#define SX126X_CMD_CLR_IRQ_STATUS         0x02
#define SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL 0x9D
#define SX126X_CMD_SET_DIO3_AS_TCXO_CTRL  0x97

#define SX126X_CMD_SET_RF_FREQUENCY       0x86
#define SX126X_CMD_SET_PKT_TYPE           0x8A
#define SX126X_CMD_GET_PKT_TYPE           0x11
#define SX126X_CMD_SET_TX_PARAMS          0x8E
#define SX126X_CMD_SET_MODULATION_PARAMS  0x8B
#define SX126X_CMD_SET_PKT_PARAMS         0x8C
#define SX126X_CMD_SET_CAD_PARAMS         0x88
#define SX126X_CMD_SET_BUFFER_BASE_ADDRESS 0x8F
#define SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT 0xA0

#define SX126X_CMD_GET_STATUS             0xC0
#define SX126X_CMD_GET_RSSI_INST          0x15
#define SX126X_CMD_GET_RX_BUFFER_STATUS   0x13
#define SX126X_CMD_GET_PKT_STATUS         0x14
#define SX126X_CMD_GET_DEVICE_ERRORS      0x17
#define SX126X_CMD_CLR_DEVICE_ERRORS      0x07
#define SX126X_CMD_GET_STATS              0x10
#define SX126X_CMD_RESET_STATS            0x00

// IRQ flags
#define SX126X_IRQ_TX_DONE                (1 << 0)
#define SX126X_IRQ_RX_DONE                (1 << 1)
#define SX126X_IRQ_PREAMBLE_DETECTED      (1 << 2)
#define SX126X_IRQ_SYNC_WORD_VALID        (1 << 3)
#define SX126X_IRQ_HEADER_VALID           (1 << 4)
#define SX126X_IRQ_HEADER_ERR             (1 << 5)
#define SX126X_IRQ_CRC_ERR                (1 << 6)
#define SX126X_IRQ_CAD_DONE               (1 << 7)
#define SX126X_IRQ_CAD_ACTIVITY_DETECTED  (1 << 8)
#define SX126X_IRQ_RX_TX_TIMEOUT          (1 << 9)

// Packet types
#define SX126X_PACKET_TYPE_GFSK           0x00
#define SX126X_PACKET_TYPE_LORA           0x01

// Standby modes
#define SX126X_STANDBY_RC                 0x00
#define SX126X_STANDBY_XOSC               0x01

// Regulator modes
#define SX126X_REGULATOR_LDO              0x00
#define SX126X_REGULATOR_DC_DC            0x01

// LoRa bandwidths
#define SX126X_LORA_BW_125                0x04
#define SX126X_LORA_BW_250                0x05
#define SX126X_LORA_BW_500                0x06

// LoRa coding rates
#define SX126X_LORA_CR_4_5                0x01
#define SX126X_LORA_CR_4_6                0x02
#define SX126X_LORA_CR_4_7                0x03
#define SX126X_LORA_CR_4_8                0x04

/**
 * @brief Initialize SX126x HAL
 * @return true on success
 */
bool sx126x_hal_init(void);

/**
 * @brief Deinitialize SX126x HAL
 */
void sx126x_hal_deinit(void);

/**
 * @brief Reset SX126x
 */
void sx126x_hal_reset(void);

/**
 * @brief Wait for BUSY pin to go low
 * @param timeout_ms Timeout in milliseconds
 * @return true if BUSY went low, false on timeout
 */
bool sx126x_hal_wait_busy(uint32_t timeout_ms);

/**
 * @brief Write command to SX126x
 * @param cmd Command byte
 * @param data Data bytes (can be NULL)
 * @param len Length of data
 */
void sx126x_hal_write_cmd(uint8_t cmd, const uint8_t *data, size_t len);

/**
 * @brief Read from SX126x
 * @param cmd Command byte
 * @param data Buffer for received data
 * @param len Length to read
 */
void sx126x_hal_read_cmd(uint8_t cmd, uint8_t *data, size_t len);

/**
 * @brief Write to buffer
 * @param offset Buffer offset
 * @param data Data to write
 * @param len Length
 */
void sx126x_hal_write_buffer(uint8_t offset, const uint8_t *data, size_t len);

/**
 * @brief Read from buffer
 * @param offset Buffer offset
 * @param data Buffer for data
 * @param len Length to read
 */
void sx126x_hal_read_buffer(uint8_t offset, uint8_t *data, size_t len);

/**
 * @brief Write register
 * @param addr Register address
 * @param data Data to write
 * @param len Length
 */
void sx126x_hal_write_reg(uint16_t addr, const uint8_t *data, size_t len);

/**
 * @brief Read register
 * @param addr Register address
 * @param data Buffer for data
 * @param len Length to read
 */
void sx126x_hal_read_reg(uint16_t addr, uint8_t *data, size_t len);

/**
 * @brief Get DIO1 pin state
 * @return true if DIO1 is high
 */
bool sx126x_hal_get_dio1(void);

/**
 * @brief Delay in milliseconds
 * @param ms Milliseconds to delay
 */
void sx126x_hal_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif
