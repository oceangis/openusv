/*
 * ArduPilot Rover ESP32-S3 Main Entry Point
 *
 * 这个文件是 ESP-IDF 的入口点，它会调用 ArduPilot 的 main() 函数
 */

#include <stdio.h>
#include <stddef.h>
#include "wifi_config.h"
#include "lora_mavlink.h"
#include "esp_log.h"

static const char *TAG = "app_main";

// ArduPilot 的 main 函数由 AP_HAL_MAIN_CALLBACKS 宏在 Rover.cpp 中生成
extern int main(int argc, char *argv[]);

void app_main(void)
{
    printf("========================================\n");
    printf("ArduPilot Rover ESP32-S3 Starting...\n");
    printf("========================================\n");

    // 初始化WiFi配置系统
    ESP_LOGI(TAG, "启动WiFi配置系统...");
    esp_err_t err = wifi_config_system_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi配置系统启动失败，继续启动ArduPilot...");
    } else {
        ESP_LOGI(TAG, "WiFi配置系统启动成功");
    }

    // 初始化LoRa MAVLink数传
    ESP_LOGI(TAG, "启动LoRa MAVLink数传...");
    lora_config_t lora_cfg = {
        .frequency = LORA_FREQ_HZ,
        .tx_power = LORA_TX_POWER,
        .spreading_factor = LORA_SF,
        .bandwidth = 0,         // 125kHz
        .coding_rate = 1,       // 4/5
        .preamble_len = 8,
        .crc_enable = true
    };
    if (lora_mavlink_init(&lora_cfg)) {
        ESP_LOGI(TAG, "LoRa MAVLink数传启动成功 (%.3f MHz, SF%d, %d dBm)",
                 lora_cfg.frequency / 1000000.0, lora_cfg.spreading_factor, lora_cfg.tx_power);
    } else {
        ESP_LOGW(TAG, "LoRa MAVLink数传启动失败，继续启动ArduPilot...");
    }

    printf("========================================\n");
    printf("启动 ArduPilot Rover...\n");
    printf("========================================\n");

    // 调用 ArduPilot 的 main 函数
    // ArduPilot 会接管控制流
    char *argv[] = {"ardurover", NULL};
    main(1, argv);

    // 不应该到达这里，因为 ArduPilot 会一直运行
    printf("ArduPilot main() returned unexpectedly!\n");
}
