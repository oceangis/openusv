/*
 * ArduPilot Rover ESP32-S3 Main Entry Point
 *
 * 这个文件是 ESP-IDF 的入口点，它会调用 ArduPilot 的 main() 函数
 */

#include <stdio.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

// ============================================================================
// 功能开关 - 调试时可禁用某些组件
// ============================================================================
#define ENABLE_WIFI_CONFIG  0   // 设为1启用WiFi配置系统
#define ENABLE_LORA_MAVLINK 1   // 设为1启用LoRa MAVLink数传

#if ENABLE_WIFI_CONFIG
#include "wifi_config.h"
#endif

#if ENABLE_LORA_MAVLINK
#include "lora_mavlink.h"
#endif

static const char *TAG = "app_main";

// ArduPilot 的 main 函数由 AP_HAL_MAIN_CALLBACKS 宏在 Rover.cpp 中生成
extern int main(int argc, char *argv[]);

void app_main(void)
{
    // 延迟一下让串口稳定
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("\n\n");
    printf("========================================\n");
    printf("ArduPilot Rover ESP32-S3 Starting...\n");
    printf("========================================\n");
    fflush(stdout);

    // !!!!! 重要：WiFi和LoRa必须在ArduPilot启动之前初始化 !!!!!
    // 这是因为：
    // 1. ArduPilot的HAL会在setup()时初始化SERIAL驱动
    // 2. 如果SERIAL1配置为WiFi(HAL_ESP32_WIFI)，会调用WiFiDriver初始化WiFi
    // 3. 我们的wifi_config系统现在可以检测并复用已初始化的WiFi
    // 4. 但最好在ArduPilot之前初始化，避免时序问题

#if ENABLE_WIFI_CONFIG
    // 初始化WiFi配置系统
    ESP_LOGI(TAG, "启动WiFi配置系统...");
    esp_err_t err = wifi_config_system_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi配置系统启动失败: %s，继续启动ArduPilot...", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "WiFi配置系统启动成功");
    }

    // 给WiFi一些时间稳定
    vTaskDelay(pdMS_TO_TICKS(500));
#else
    ESP_LOGI(TAG, "WiFi配置系统已禁用");
#endif

#if ENABLE_LORA_MAVLINK
    // 初始化LoRa MAVLink数传
    ESP_LOGI(TAG, "启动LoRa MAVLink数传...");
    lora_config_t lora_cfg = {
        .frequency = LORA_FREQ_HZ,
        .tx_power = LORA_TX_POWER,
        .spreading_factor = LORA_SF,
        .bandwidth = 2,         // 500kHz (0=125k, 1=250k, 2=500k) - 匹配 E22-400MBL
        .coding_rate = 1,       // 4/5
        .preamble_len = 8,
        .crc_enable = true
    };
    if (lora_mavlink_init(&lora_cfg)) {
        ESP_LOGI(TAG, "LoRa MAVLink数传启动成功 (%.3f MHz, SF%d, %d dBm)",
                 lora_cfg.frequency / 1000000.0, lora_cfg.spreading_factor, lora_cfg.tx_power);

        // 发送测试包验证通信 (EBYTE 格式)
        for (int i = 1; i <= 3; i++) {
            char test_msg[32];
            int len = snprintf(test_msg, sizeof(test_msg), "TX.003.%03d", i);
            lora_mavlink_write((uint8_t*)test_msg, len);
            ESP_LOGI(TAG, "LoRa 测试包 #%d: %s", i, test_msg);
            vTaskDelay(pdMS_TO_TICKS(1500));  // 等待发送完成
        }
        ESP_LOGI(TAG, "LoRa 测试完成");
    } else {
        ESP_LOGW(TAG, "LoRa MAVLink数传启动失败，继续启动ArduPilot...");
    }

    // 给LoRa一些时间稳定
    vTaskDelay(pdMS_TO_TICKS(200));
#else
    ESP_LOGI(TAG, "LoRa MAVLink数传已禁用");
#endif

    printf("========================================\n");
    printf("启动 ArduPilot Rover...\n");
    printf("========================================\n");
    fflush(stdout);

    // 调用 ArduPilot 的 main 函数
    // ArduPilot 会接管控制流
    char *argv[] = {"ardurover", NULL};
    main(1, argv);

    // 不应该到达这里，因为 ArduPilot 会一直运行
    printf("ArduPilot main() returned unexpectedly!\n");
}
