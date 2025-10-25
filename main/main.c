/*
 * ArduPilot Rover ESP32-S3 Main Entry Point
 *
 * 这个文件是 ESP-IDF 的入口点，它会调用 ArduPilot 的 main() 函数
 */

#include <stdio.h>
#include <stddef.h>

// ArduPilot 的 main 函数由 AP_HAL_MAIN_CALLBACKS 宏在 Rover.cpp 中生成
extern int main(int argc, char *argv[]);

void app_main(void)
{
    printf("ArduPilot Rover ESP32-S3 Starting...\n");

    // 调用 ArduPilot 的 main 函数
    // ArduPilot 会接管控制流
    char *argv[] = {"ardurover", NULL};
    main(1, argv);

    // 不应该到达这里，因为 ArduPilot 会一直运行
    printf("ArduPilot main() returned unexpectedly!\n");
}
