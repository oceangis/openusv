/**
 * @file sx126x.c
 * @brief SX1262 LoRa radio driver implementation
 */

#include "sx126x_hal.h"
#include "lora_mavlink.h"

#include <string.h>

// SX1262 register addresses
#define SX126X_REG_LORA_SYNC_WORD_MSB     0x0740
#define SX126X_REG_LORA_SYNC_WORD_LSB     0x0741
#define SX126X_REG_TX_CLAMP_CONFIG        0x08D8
#define SX126X_REG_OCP_CONFIGURATION      0x08E7
#define SX126X_REG_RTC_CONTROL            0x0902
#define SX126X_REG_XTA_TRIM               0x0911
#define SX126X_REG_XTB_TRIM               0x0912
#define SX126X_REG_DIO3_OUTPUT_VOLTAGE    0x0920
#define SX126X_REG_EVENT_MASK             0x0944

// LoRa sync word values
#define SX126X_LORA_SYNC_WORD_PUBLIC      0x3444
#define SX126X_LORA_SYNC_WORD_PRIVATE     0x1424
// EBYTE E22 模块使用的 Sync Word (sx126x_set_lora_sync_word(0x14) -> 0x1444)
#define SX126X_LORA_SYNC_WORD_EBYTE       0x1444

// PA config constants for SX1262
#define SX1262_PA_DUTY_CYCLE              0x04
#define SX1262_HP_MAX                     0x07
#define SX1262_DEVICE_SEL                 0x00
#define SX1262_PA_LUT                     0x01

// Ramp times
#define SX126X_RAMP_10_US                 0x00
#define SX126X_RAMP_20_US                 0x01
#define SX126X_RAMP_40_US                 0x02
#define SX126X_RAMP_80_US                 0x03
#define SX126X_RAMP_200_US                0x04
#define SX126X_RAMP_800_US                0x05
#define SX126X_RAMP_1700_US               0x06
#define SX126X_RAMP_3400_US               0x07

// CAD symbol values
#define SX126X_CAD_ON_1_SYMB              0x00
#define SX126X_CAD_ON_2_SYMB              0x01
#define SX126X_CAD_ON_4_SYMB              0x02
#define SX126X_CAD_ON_8_SYMB              0x03
#define SX126X_CAD_ON_16_SYMB             0x04

// CAD exit modes
#define SX126X_CAD_ONLY                   0x00
#define SX126X_CAD_RX                     0x01

// Timeout values
#define SX126X_RX_CONTINUOUS              0xFFFFFF
#define SX126X_TX_TIMEOUT_NONE            0x000000

// Static variables
static bool sx126x_initialized = false;
static uint8_t current_sf = 8;
static uint8_t current_bw = SX126X_LORA_BW_125;
static uint8_t current_cr = SX126X_LORA_CR_4_5;

/**
 * @brief Set SX126x to standby mode
 * @param mode Standby mode (RC or XOSC)
 */
void sx126x_set_standby(uint8_t mode)
{
    sx126x_hal_write_cmd(SX126X_CMD_SET_STANDBY, &mode, 1);
}

/**
 * @brief Set packet type (GFSK or LoRa)
 * @param type Packet type
 */
void sx126x_set_packet_type(uint8_t type)
{
    sx126x_hal_write_cmd(SX126X_CMD_SET_PKT_TYPE, &type, 1);
}

/**
 * @brief Set RF frequency
 * @param freq_hz Frequency in Hz
 */
void sx126x_set_rf_frequency(uint32_t freq_hz)
{
    // freq_reg = freq_hz * 2^25 / 32MHz
    uint32_t freq_reg = (uint32_t)((double)freq_hz / (double)(32000000) * (double)(1 << 25));

    uint8_t buf[4];
    buf[0] = (freq_reg >> 24) & 0xFF;
    buf[1] = (freq_reg >> 16) & 0xFF;
    buf[2] = (freq_reg >> 8) & 0xFF;
    buf[3] = freq_reg & 0xFF;

    sx126x_hal_write_cmd(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);
}

/**
 * @brief Configure PA for SX1262
 * @param power_dbm Output power in dBm
 */
void sx126x_set_pa_config(int8_t power_dbm)
{
    uint8_t buf[4];

    // SX1262 PA configuration
    buf[0] = SX1262_PA_DUTY_CYCLE;  // paDutyCycle
    buf[1] = SX1262_HP_MAX;         // hpMax
    buf[2] = SX1262_DEVICE_SEL;     // deviceSel (0 for SX1262)
    buf[3] = SX1262_PA_LUT;         // paLut

    sx126x_hal_write_cmd(SX126X_CMD_SET_PA_CONFIG, buf, 4);
}

/**
 * @brief Set TX parameters
 * @param power_dbm Output power in dBm (-9 to +22)
 * @param ramp_time Ramp time
 */
void sx126x_set_tx_params(int8_t power_dbm, uint8_t ramp_time)
{
    // Clamp power to valid range for SX1262
    if (power_dbm > 22) power_dbm = 22;
    if (power_dbm < -9) power_dbm = -9;

    uint8_t buf[2];
    buf[0] = (uint8_t)power_dbm;
    buf[1] = ramp_time;

    sx126x_hal_write_cmd(SX126X_CMD_SET_TX_PARAMS, buf, 2);
}

/**
 * @brief Set buffer base addresses
 * @param tx_base TX buffer base address
 * @param rx_base RX buffer base address
 */
void sx126x_set_buffer_base_address(uint8_t tx_base, uint8_t rx_base)
{
    uint8_t buf[2] = {tx_base, rx_base};
    sx126x_hal_write_cmd(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2);
}

/**
 * @brief Convert bandwidth value to SX126x register value
 * @param bw_khz Bandwidth in kHz (125, 250, 500)
 * @return Register value
 */
static uint8_t bandwidth_to_reg(uint16_t bw_khz)
{
    switch (bw_khz) {
        case 125: return SX126X_LORA_BW_125;
        case 250: return SX126X_LORA_BW_250;
        case 500: return SX126X_LORA_BW_500;
        default:  return SX126X_LORA_BW_125;
    }
}

/**
 * @brief Set LoRa modulation parameters
 * @param sf Spreading factor (5-12)
 * @param bw Bandwidth register value
 * @param cr Coding rate (1-4)
 * @param ldro Low data rate optimization (0=off, 1=on)
 */
void sx126x_set_modulation_params(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
    uint8_t buf[4];
    buf[0] = sf;
    buf[1] = bw;
    buf[2] = cr;
    buf[3] = ldro;

    current_sf = sf;
    current_bw = bw;
    current_cr = cr;

    sx126x_hal_write_cmd(SX126X_CMD_SET_MODULATION_PARAMS, buf, 4);
}

/**
 * @brief Set LoRa packet parameters
 * @param preamble_len Preamble length
 * @param header_type 0=Variable length (explicit header), 1=Fixed length (implicit header)
 * @param payload_len Payload length
 * @param crc_enable CRC enable
 * @param invert_iq Invert IQ
 */
void sx126x_set_packet_params(uint16_t preamble_len, uint8_t header_type,
                               uint8_t payload_len, uint8_t crc_enable, uint8_t invert_iq)
{
    uint8_t buf[6];
    buf[0] = (preamble_len >> 8) & 0xFF;
    buf[1] = preamble_len & 0xFF;
    buf[2] = header_type;
    buf[3] = payload_len;
    buf[4] = crc_enable;
    buf[5] = invert_iq;

    sx126x_hal_write_cmd(SX126X_CMD_SET_PKT_PARAMS, buf, 6);
}

/**
 * @brief Set DIO and IRQ parameters
 * @param irq_mask IRQ mask
 * @param dio1_mask DIO1 mask
 * @param dio2_mask DIO2 mask
 * @param dio3_mask DIO3 mask
 */
void sx126x_set_dio_irq_params(uint16_t irq_mask, uint16_t dio1_mask,
                                uint16_t dio2_mask, uint16_t dio3_mask)
{
    uint8_t buf[8];
    buf[0] = (irq_mask >> 8) & 0xFF;
    buf[1] = irq_mask & 0xFF;
    buf[2] = (dio1_mask >> 8) & 0xFF;
    buf[3] = dio1_mask & 0xFF;
    buf[4] = (dio2_mask >> 8) & 0xFF;
    buf[5] = dio2_mask & 0xFF;
    buf[6] = (dio3_mask >> 8) & 0xFF;
    buf[7] = dio3_mask & 0xFF;

    sx126x_hal_write_cmd(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}

/**
 * @brief Get IRQ status
 * @return IRQ status flags
 */
uint16_t sx126x_get_irq_status(void)
{
    uint8_t buf[2];
    sx126x_hal_read_cmd(SX126X_CMD_GET_IRQ_STATUS, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

/**
 * @brief Clear IRQ status
 * @param irq_mask IRQ flags to clear
 */
void sx126x_clear_irq_status(uint16_t irq_mask)
{
    uint8_t buf[2];
    buf[0] = (irq_mask >> 8) & 0xFF;
    buf[1] = irq_mask & 0xFF;
    sx126x_hal_write_cmd(SX126X_CMD_CLR_IRQ_STATUS, buf, 2);
}

/**
 * @brief Set DIO2 as RF switch control
 * @param enable true to enable
 */
void sx126x_set_dio2_rf_switch(bool enable)
{
    uint8_t val = enable ? 1 : 0;
    sx126x_hal_write_cmd(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &val, 1);
}

/**
 * @brief Set regulator mode
 * @param mode 0=LDO, 1=DC-DC
 */
void sx126x_set_regulator_mode(uint8_t mode)
{
    sx126x_hal_write_cmd(SX126X_CMD_SET_REGULATOR_MODE, &mode, 1);
}

/**
 * @brief Calibrate the radio
 * @param calib_param Calibration parameters
 */
void sx126x_calibrate(uint8_t calib_param)
{
    sx126x_hal_write_cmd(SX126X_CMD_CALIBRATE, &calib_param, 1);
    sx126x_hal_delay_ms(10);
}

/**
 * @brief Calibrate image rejection
 * @param freq1 Frequency parameter 1
 * @param freq2 Frequency parameter 2
 */
void sx126x_calibrate_image(uint8_t freq1, uint8_t freq2)
{
    uint8_t buf[2] = {freq1, freq2};
    sx126x_hal_write_cmd(SX126X_CMD_CALIBRATE_IMAGE, buf, 2);
}

/**
 * @brief Set LoRa sync word
 * @param sync_word Sync word value
 */
void sx126x_set_sync_word(uint16_t sync_word)
{
    uint8_t buf[2];
    buf[0] = (sync_word >> 8) & 0xFF;
    buf[1] = sync_word & 0xFF;
    sx126x_hal_write_reg(SX126X_REG_LORA_SYNC_WORD_MSB, buf, 2);
}

/**
 * @brief Start transmission
 * @param timeout Timeout in 15.625us steps (0 = no timeout)
 */
void sx126x_set_tx(uint32_t timeout)
{
    uint8_t buf[3];
    buf[0] = (timeout >> 16) & 0xFF;
    buf[1] = (timeout >> 8) & 0xFF;
    buf[2] = timeout & 0xFF;

    sx126x_hal_write_cmd(SX126X_CMD_SET_TX, buf, 3);
}

/**
 * @brief Start reception
 * @param timeout Timeout in 15.625us steps (0xFFFFFF = continuous)
 */
void sx126x_set_rx(uint32_t timeout)
{
    uint8_t buf[3];
    buf[0] = (timeout >> 16) & 0xFF;
    buf[1] = (timeout >> 8) & 0xFF;
    buf[2] = timeout & 0xFF;

    sx126x_hal_write_cmd(SX126X_CMD_SET_RX, buf, 3);
}

/**
 * @brief Set CAD parameters
 * @param cad_symbol_num Number of symbols for CAD
 * @param cad_det_peak CAD detection peak
 * @param cad_det_min CAD detection minimum
 * @param cad_exit_mode CAD exit mode
 * @param cad_timeout CAD timeout
 */
void sx126x_set_cad_params(uint8_t cad_symbol_num, uint8_t cad_det_peak,
                           uint8_t cad_det_min, uint8_t cad_exit_mode, uint32_t cad_timeout)
{
    uint8_t buf[7];
    buf[0] = cad_symbol_num;
    buf[1] = cad_det_peak;
    buf[2] = cad_det_min;
    buf[3] = cad_exit_mode;
    buf[4] = (cad_timeout >> 16) & 0xFF;
    buf[5] = (cad_timeout >> 8) & 0xFF;
    buf[6] = cad_timeout & 0xFF;

    sx126x_hal_write_cmd(SX126X_CMD_SET_CAD_PARAMS, buf, 7);
}

/**
 * @brief Start CAD operation
 */
void sx126x_set_cad(void)
{
    sx126x_hal_write_cmd(SX126X_CMD_SET_CAD, NULL, 0);
}

/**
 * @brief Get RX buffer status
 * @param payload_len Pointer to store payload length
 * @param rx_start_ptr Pointer to store RX buffer start pointer
 */
void sx126x_get_rx_buffer_status(uint8_t *payload_len, uint8_t *rx_start_ptr)
{
    uint8_t buf[2];
    sx126x_hal_read_cmd(SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 2);

    if (payload_len) *payload_len = buf[0];
    if (rx_start_ptr) *rx_start_ptr = buf[1];
}

/**
 * @brief Get packet status
 * @param rssi Pointer to store RSSI (dBm = -rssi/2)
 * @param snr Pointer to store SNR (dB = snr/4)
 * @param signal_rssi Pointer to store signal RSSI
 */
void sx126x_get_packet_status(int16_t *rssi, int8_t *snr, int16_t *signal_rssi)
{
    uint8_t buf[3];
    sx126x_hal_read_cmd(SX126X_CMD_GET_PKT_STATUS, buf, 3);

    if (rssi) *rssi = -((int16_t)buf[0]) / 2;
    if (snr) *snr = ((int8_t)buf[1]) / 4;
    if (signal_rssi) *signal_rssi = -((int16_t)buf[2]) / 2;
}

/**
 * @brief Get device status
 * @return Status byte
 */
uint8_t sx126x_get_status(void)
{
    uint8_t status;
    sx126x_hal_read_cmd(SX126X_CMD_GET_STATUS, &status, 1);
    return status;
}

/**
 * @brief Get device errors
 * @return Error flags
 */
uint16_t sx126x_get_device_errors(void)
{
    uint8_t buf[2];
    sx126x_hal_read_cmd(SX126X_CMD_GET_DEVICE_ERRORS, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

/**
 * @brief Clear device errors
 */
void sx126x_clear_device_errors(void)
{
    uint8_t buf[2] = {0, 0};
    sx126x_hal_write_cmd(SX126X_CMD_CLR_DEVICE_ERRORS, buf, 2);
}

/**
 * @brief Set sleep mode
 * @param config Sleep configuration
 */
void sx126x_set_sleep(uint8_t config)
{
    sx126x_hal_write_cmd(SX126X_CMD_SET_SLEEP, &config, 1);
}

/**
 * @brief Write data to TX buffer
 * @param data Data to write
 * @param len Length of data
 */
void sx126x_write_buffer(const uint8_t *data, size_t len)
{
    sx126x_hal_write_buffer(0, data, len);
}

/**
 * @brief Read data from RX buffer
 * @param data Buffer to store data
 * @param len Length to read
 * @param offset Buffer offset
 */
void sx126x_read_buffer(uint8_t *data, size_t len, uint8_t offset)
{
    sx126x_hal_read_buffer(offset, data, len);
}

/**
 * @brief Wakeup SX126x from sleep (软件唤醒，用于无 RST 引脚的情况)
 */
static void sx126x_wakeup(void)
{
    // 发送 GetStatus 命令唤醒芯片
    uint8_t status;
    sx126x_hal_read_cmd(SX126X_CMD_GET_STATUS, &status, 1);
    sx126x_hal_delay_ms(10);
}

/**
 * @brief Initialize SX1262 for LoRa operation
 * @param config LoRa configuration
 * @return true on success
 */
bool sx126x_init(const lora_config_t *config)
{
    if (sx126x_initialized) {
        return true;
    }

    // Initialize HAL
    if (!sx126x_hal_init()) {
        return false;
    }

    // Reset the radio (or wait if no RST pin)
    sx126x_hal_reset();

    // 软件唤醒 (因为 RST 引脚未连接)
    sx126x_wakeup();

    // Wait for chip ready
    if (!sx126x_hal_wait_busy(100)) {
        return false;
    }

    // Set standby mode (RC oscillator)
    sx126x_set_standby(SX126X_STANDBY_RC);

    // Set packet type to LoRa
    sx126x_set_packet_type(SX126X_PACKET_TYPE_LORA);

    // Set regulator mode to DC-DC for better efficiency
    sx126x_set_regulator_mode(SX126X_REGULATOR_DC_DC);

    // Calibrate all blocks
    sx126x_calibrate(0x7F);

    // Calibrate image for 433 MHz band (430-440 MHz)
    uint32_t freq = config ? config->frequency : LORA_FREQ_HZ;
    if (freq >= 430000000 && freq <= 440000000) {
        sx126x_calibrate_image(0x6B, 0x6F);
    } else if (freq >= 470000000 && freq <= 510000000) {
        sx126x_calibrate_image(0x75, 0x81);
    } else if (freq >= 863000000 && freq <= 870000000) {
        sx126x_calibrate_image(0xD7, 0xDB);
    } else if (freq >= 902000000 && freq <= 928000000) {
        sx126x_calibrate_image(0xE1, 0xE9);
    }

    // Set DIO2 as RF switch control
    sx126x_set_dio2_rf_switch(true);

    // Set buffer base addresses (TX at 0, RX at 128)
    sx126x_set_buffer_base_address(0, 128);

    // Configure PA
    int8_t tx_power = config ? config->tx_power : LORA_TX_POWER;
    sx126x_set_pa_config(tx_power);

    // Set OCP (过流保护) - 与 EBYTE 官方 demo 一致
    // 0x38 = 140mA (SX1262 推荐值为 140mA @ 22dBm)
    uint8_t ocp_val = 0x38;
    sx126x_hal_write_reg(SX126X_REG_OCP_CONFIGURATION, &ocp_val, 1);

    // Set TX parameters
    sx126x_set_tx_params(tx_power, SX126X_RAMP_40_US);  // 40us ramp 与测试一致

    // Set frequency
    sx126x_set_rf_frequency(freq);

    // Set modulation parameters
    uint8_t sf = config ? config->spreading_factor : LORA_SF;
    // Bandwidth: config->bandwidth is 0=125kHz, 1=250kHz, 2=500kHz
    uint8_t bw;
    if (config) {
        switch (config->bandwidth) {
            case 0:  bw = SX126X_LORA_BW_125; break;
            case 1:  bw = SX126X_LORA_BW_250; break;
            case 2:
            default: bw = SX126X_LORA_BW_500; break;
        }
    } else {
        bw = SX126X_LORA_BW_500;  // Default 500kHz for E22-400MBL compatibility
    }
    uint8_t cr = config ? config->coding_rate : SX126X_LORA_CR_4_5;

    // Calculate LDRO: required if symbol time > 16ms
    // Symbol time = 2^SF / BW
    uint8_t ldro = 0;
    if (sf >= 10) {
        ldro = 1;  // Enable LDRO for SF10, SF11, SF12
    }

    sx126x_set_modulation_params(sf, bw, cr, ldro);

    // Set packet parameters
    uint16_t preamble = config ? config->preamble_len : 8;
    uint8_t crc = config ? (config->crc_enable ? 1 : 0) : 1;
    sx126x_set_packet_params(preamble, 0, 255, crc, 0);

    // Set sync word - 使用 EBYTE E22 模块的 Sync Word (0x1444)
    // EBYTE 调用 sx126x_set_lora_sync_word(0x14) 实际产生 0x1444
    // 经测试验证与 E22-400MBL-SC 评估板通信正常
    sx126x_set_sync_word(SX126X_LORA_SYNC_WORD_EBYTE);

    // Configure IRQs on DIO1
    uint16_t irq_mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE |
                        SX126X_IRQ_CRC_ERR | SX126X_IRQ_RX_TX_TIMEOUT |
                        SX126X_IRQ_CAD_DONE | SX126X_IRQ_CAD_ACTIVITY_DETECTED;
    sx126x_set_dio_irq_params(irq_mask, irq_mask, 0, 0);

    // Clear any pending errors
    sx126x_clear_device_errors();
    sx126x_clear_irq_status(0xFFFF);

    sx126x_initialized = true;
    return true;
}

/**
 * @brief Deinitialize SX1262
 */
void sx126x_deinit(void)
{
    if (!sx126x_initialized) {
        return;
    }

    // Put radio to sleep
    sx126x_set_sleep(0x00);

    // Deinitialize HAL
    sx126x_hal_deinit();

    sx126x_initialized = false;
}

/**
 * @brief Check if SX1262 is initialized
 * @return true if initialized
 */
bool sx126x_is_initialized(void)
{
    return sx126x_initialized;
}

/**
 * @brief Send a packet
 * @param data Data to send
 * @param len Length of data
 * @return true if transmission started
 */
bool sx126x_send_packet(const uint8_t *data, size_t len)
{
    if (!sx126x_initialized || !data || len == 0 || len > LORA_MAX_PACKET_SIZE) {
        return false;
    }

    // Set standby
    sx126x_set_standby(SX126X_STANDBY_RC);

    // Update packet length in packet params
    sx126x_set_packet_params(8, 0, len, 1, 0);

    // Write data to buffer
    sx126x_write_buffer(data, len);

    // Clear IRQ
    sx126x_clear_irq_status(0xFFFF);

    // Start TX (no timeout)
    sx126x_set_tx(SX126X_TX_TIMEOUT_NONE);

    return true;
}

/**
 * @brief Start continuous reception
 * @return true if RX started
 */
bool sx126x_start_rx_continuous(void)
{
    if (!sx126x_initialized) {
        return false;
    }

    // Set standby
    sx126x_set_standby(SX126X_STANDBY_RC);

    // Clear IRQ
    sx126x_clear_irq_status(0xFFFF);

    // Start continuous RX
    sx126x_set_rx(SX126X_RX_CONTINUOUS);

    return true;
}

/**
 * @brief Receive a packet
 * @param data Buffer for received data
 * @param max_len Maximum length to receive
 * @param rssi Pointer to store RSSI (can be NULL)
 * @param snr Pointer to store SNR (can be NULL)
 * @return Length of received data, 0 if no data
 */
size_t sx126x_receive_packet(uint8_t *data, size_t max_len, int16_t *rssi, int8_t *snr)
{
    if (!sx126x_initialized || !data) {
        return 0;
    }

    // Get RX buffer status
    uint8_t payload_len = 0;
    uint8_t rx_start = 0;
    sx126x_get_rx_buffer_status(&payload_len, &rx_start);

    if (payload_len == 0 || payload_len > max_len) {
        return 0;
    }

    // Read data from buffer
    sx126x_read_buffer(data, payload_len, rx_start);

    // Get packet status
    int16_t signal_rssi;
    sx126x_get_packet_status(rssi, snr, &signal_rssi);

    return payload_len;
}

/**
 * @brief Start CAD (Channel Activity Detection)
 * @return true if CAD started
 */
bool sx126x_start_cad(void)
{
    if (!sx126x_initialized) {
        return false;
    }

    // Set standby
    sx126x_set_standby(SX126X_STANDBY_RC);

    // Configure CAD parameters based on SF
    uint8_t cad_det_peak, cad_det_min;

    // Recommended values from Semtech AN1200.48
    switch (current_sf) {
        case 5:
            cad_det_peak = 18;
            cad_det_min = 10;
            break;
        case 6:
            cad_det_peak = 19;
            cad_det_min = 10;
            break;
        case 7:
            cad_det_peak = 20;
            cad_det_min = 10;
            break;
        case 8:
            cad_det_peak = 21;
            cad_det_min = 10;
            break;
        case 9:
            cad_det_peak = 22;
            cad_det_min = 10;
            break;
        case 10:
            cad_det_peak = 23;
            cad_det_min = 10;
            break;
        case 11:
            cad_det_peak = 24;
            cad_det_min = 10;
            break;
        case 12:
            cad_det_peak = 25;
            cad_det_min = 10;
            break;
        default:
            cad_det_peak = 21;
            cad_det_min = 10;
            break;
    }

    sx126x_set_cad_params(SX126X_CAD_ON_4_SYMB, cad_det_peak, cad_det_min,
                          SX126X_CAD_ONLY, 0);

    // Clear IRQ
    sx126x_clear_irq_status(0xFFFF);

    // Start CAD
    sx126x_set_cad();

    return true;
}
