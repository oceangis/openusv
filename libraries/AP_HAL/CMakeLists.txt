# ArduPilot AP_HAL 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "CANIface.cpp" "Device.cpp" "DSP.cpp" "GPIO.cpp" "HAL.cpp" "RCOutput.cpp" "Scheduler.cpp" "Semaphores.cpp" "SIMState.cpp" "Storage.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "board" "hwdef" "hwdef/scripts" "utility" "../AP_HAL_ESP32/hwdef"  # 主要包含目录，添加 ESP32 hwdef 路径
    REQUIRES freertos esp_timer driver AP_HAL_ESP32
)

# 完整源文件列表
set(COMPONENT_SRCS
    "CANIface.cpp"
    "Device.cpp"
    "DSP.cpp"
    "GPIO.cpp"
    "HAL.cpp"
    "RCOutput.cpp"
    "Scheduler.cpp"
    "Semaphores.cpp"
    "SIMState.cpp"
    "Storage.cpp"
    "system.cpp"
    "UARTDriver.cpp"
    "Util.cpp"
    "CANIface.cpp"
    "Device.cpp"
    "DSP.cpp"
    "GPIO.cpp"
    "HAL.cpp"
    "RCOutput.cpp"
    "Scheduler.cpp"
    "Semaphores.cpp"
    "SIMState.cpp"
    "Storage.cpp"
    "system.cpp"
    "UARTDriver.cpp"
    "Util.cpp"
    "utility/BetterStream.cpp"
    "utility/DataRateLimit.cpp"
    "utility/dsm.cpp"
    "utility/ftoa_engine.cpp"
    "utility/getopt_cpp.cpp"
    "utility/packetise.cpp"
    "utility/print_vprintf.cpp"
    "utility/RCOutput_Tap.cpp"
    "utility/replace.cpp"
    "utility/RingBuffer.cpp"
    "utility/Socket.cpp"
    "utility/Socket_native.cpp"
    "utility/srxl.cpp"
    "utility/st24.cpp"
    "utility/sumd.cpp"
    "utility/utoa_invert.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "board"
    "hwdef"
    "hwdef/scripts"
    "utility"
)
