/**
 * @file sx126x_hal_esp32.c
 * @brief SX126x HAL implementation for ESP32-S3
 */

#include "sx126x_hal.h"
#include "lora_mavlink.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

// SPI handle
static spi_device_handle_t spi_handle = NULL;
static bool hal_initialized = false;

// Pin definitions from hwdef
static const gpio_num_t PIN_SCK  = LORA_PIN_SCK;
static const gpio_num_t PIN_MISO = LORA_PIN_MISO;
static const gpio_num_t PIN_MOSI = LORA_PIN_MOSI;
static const gpio_num_t PIN_CS   = LORA_PIN_CS;
static const gpio_num_t PIN_RST  = LORA_PIN_RST;
static const gpio_num_t PIN_BUSY = LORA_PIN_BUSY;
static const gpio_num_t PIN_DIO1 = LORA_PIN_DIO1;

bool sx126x_hal_init(void)
{
    if (hal_initialized) {
        return true;
    }

    // Configure GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_RST) | (1ULL << PIN_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << PIN_BUSY) | (1ULL << PIN_DIO1);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Set initial pin states
    gpio_set_level(PIN_CS, 1);
    gpio_set_level(PIN_RST, 1);

    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        return false;
    }

    // Configure SPI device
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 8 * 1000 * 1000,  // 8 MHz
        .mode = 0,                           // SPI mode 0
        .spics_io_num = -1,                  // Manual CS control
        .queue_size = 1,
        .flags = 0,
        .pre_cb = NULL,
        .post_cb = NULL
    };

    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_handle);
    if (ret != ESP_OK) {
        spi_bus_free(SPI2_HOST);
        return false;
    }

    hal_initialized = true;
    return true;
}

void sx126x_hal_deinit(void)
{
    if (!hal_initialized) {
        return;
    }

    if (spi_handle) {
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
    }
    spi_bus_free(SPI2_HOST);

    hal_initialized = false;
}

void sx126x_hal_reset(void)
{
    gpio_set_level(PIN_RST, 0);
    sx126x_hal_delay_ms(10);
    gpio_set_level(PIN_RST, 1);
    sx126x_hal_delay_ms(20);
}

bool sx126x_hal_wait_busy(uint32_t timeout_ms)
{
    uint32_t start = (uint32_t)(esp_timer_get_time() / 1000);

    while (gpio_get_level(PIN_BUSY)) {
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000) - start;
        if (elapsed > timeout_ms) {
            return false;
        }
        vTaskDelay(1);
    }
    return true;
}

static void spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
    if (!spi_handle || len == 0) {
        return;
    }

    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
        .flags = 0
    };

    gpio_set_level(PIN_CS, 0);
    spi_device_polling_transmit(spi_handle, &trans);
    gpio_set_level(PIN_CS, 1);
}

void sx126x_hal_write_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    if (!sx126x_hal_wait_busy(100)) {
        return;
    }

    uint8_t tx_buf[257];
    tx_buf[0] = cmd;
    if (data && len > 0) {
        memcpy(&tx_buf[1], data, len);
    }

    spi_transfer(tx_buf, NULL, len + 1);
}

void sx126x_hal_read_cmd(uint8_t cmd, uint8_t *data, size_t len)
{
    if (!sx126x_hal_wait_busy(100)) {
        return;
    }

    uint8_t tx_buf[258];
    uint8_t rx_buf[258];

    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = cmd;

    // Command + NOP + data
    spi_transfer(tx_buf, rx_buf, len + 2);

    if (data) {
        memcpy(data, &rx_buf[2], len);
    }
}

void sx126x_hal_write_buffer(uint8_t offset, const uint8_t *data, size_t len)
{
    if (!sx126x_hal_wait_busy(100)) {
        return;
    }

    uint8_t tx_buf[258];
    tx_buf[0] = SX126X_CMD_WRITE_BUFFER;
    tx_buf[1] = offset;
    if (data && len > 0) {
        memcpy(&tx_buf[2], data, len);
    }

    spi_transfer(tx_buf, NULL, len + 2);
}

void sx126x_hal_read_buffer(uint8_t offset, uint8_t *data, size_t len)
{
    if (!sx126x_hal_wait_busy(100)) {
        return;
    }

    uint8_t tx_buf[259];
    uint8_t rx_buf[259];

    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = SX126X_CMD_READ_BUFFER;
    tx_buf[1] = offset;

    // Cmd + offset + NOP + data
    spi_transfer(tx_buf, rx_buf, len + 3);

    if (data) {
        memcpy(data, &rx_buf[3], len);
    }
}

void sx126x_hal_write_reg(uint16_t addr, const uint8_t *data, size_t len)
{
    if (!sx126x_hal_wait_busy(100)) {
        return;
    }

    uint8_t tx_buf[259];
    tx_buf[0] = SX126X_CMD_WRITE_REGISTER;
    tx_buf[1] = (addr >> 8) & 0xFF;
    tx_buf[2] = addr & 0xFF;
    if (data && len > 0) {
        memcpy(&tx_buf[3], data, len);
    }

    spi_transfer(tx_buf, NULL, len + 3);
}

void sx126x_hal_read_reg(uint16_t addr, uint8_t *data, size_t len)
{
    if (!sx126x_hal_wait_busy(100)) {
        return;
    }

    uint8_t tx_buf[260];
    uint8_t rx_buf[260];

    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = SX126X_CMD_READ_REGISTER;
    tx_buf[1] = (addr >> 8) & 0xFF;
    tx_buf[2] = addr & 0xFF;

    // Cmd + addr(2) + NOP + data
    spi_transfer(tx_buf, rx_buf, len + 4);

    if (data) {
        memcpy(data, &rx_buf[4], len);
    }
}

bool sx126x_hal_get_dio1(void)
{
    return gpio_get_level(PIN_DIO1) != 0;
}

void sx126x_hal_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}
