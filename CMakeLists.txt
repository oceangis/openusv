# ArduPilot Rover ESP32-S3 - 纯 ESP-IDF 项目
# 完全基于 ESP-IDF CMake 系统，不使用 WAF

cmake_minimum_required(VERSION 3.16)

# 设置 ArduPilot HAL 板型配置
# ESP32-S3 板型选项:
#   6007 = HAL_BOARD_SUBTYPE_ESP32_S3DEVKIT   (完整功能，支持 MPU9250/AK8963/BMP280)
#   6008 = HAL_BOARD_SUBTYPE_ESP32_S3EMPTY    (最小化配置，无传感器)
#   6009 = HAL_BOARD_SUBTYPE_ESP32_S3M5STAMPFLY (M5StampFly 专用)
add_compile_definitions(CONFIG_HAL_BOARD_SUBTYPE=6007)
add_compile_definitions(HAL_BOARD_ESP32=12)

# 设置额外组件目录
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components"
)

# 包含 ESP-IDF 项目系统
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 定义项目
project(ardupilot_rover_esp32s3)
