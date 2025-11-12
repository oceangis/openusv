# ArduPilot Rover ESP32-S3 - 纯 ESP-IDF 项目
# 完全基于 ESP-IDF CMake 系统，不使用 WAF

cmake_minimum_required(VERSION 3.16)

# ===========================================================================
# Pre-Build Automation
# Must be executed before project() to ensure all dependencies are ready
# ===========================================================================

# Load pre-build script
include("${CMAKE_CURRENT_LIST_DIR}/cmake/PreBuild.cmake")

# Execute pre-build automation
ardupilot_prebuild()

# ===========================================================================
# ArduPilot Configuration
# ===========================================================================

# 设置 ArduPilot HAL 板型配置
# 注意: CONFIG_HAL_BOARD_SUBTYPE 在 ESP32 上只用于通过编译检查，
#       实际硬件配置由 libraries/AP_HAL_ESP32/hwdef/hwdef.h 决定
# ESP32-S3 板型编号 (仅用于编译检查):
#   6008 = HAL_BOARD_SUBTYPE_ESP32_S3EMPTY (通用ESP32-S3，通过hwdef.h配置硬件)
# 实际板型: esp32s3_icm20948 (在 hwdef.h 中定义)
#   - ICM20948 IMU on I2C (GPIO20/21)
#   - AK09916 Compass (内置于ICM20948)
#   - 无SD卡，无文件系统
add_compile_definitions(CONFIG_HAL_BOARD_SUBTYPE=6008)
add_compile_definitions(HAL_BOARD_ESP32=12)
add_compile_definitions(AP_SCRIPTING_ENABLED=0)

# DroneCAN C++ 包装器 - 启用
# ArduPilot 的 libcanard Subscriber<T> 模板需要 C++ wrapper
# cxx_wrappers.h 中的宏会生成 xxx_cxx_iface 类实现
add_compile_definitions(DRONECAN_CXX_WRAPPERS)

# 设置额外组件目录
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components"
)

# 包含 ESP-IDF 项目系统
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 定义项目
project(ardupilot_rover_esp32s3)
