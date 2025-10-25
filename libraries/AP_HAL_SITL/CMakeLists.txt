# ArduPilot AP_HAL_SITL 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AnalogIn.cpp" "CANSocketIface.cpp" "CAN_Multicast.cpp" "CAN_SocketCAN.cpp" "DSP.cpp" "GPIO.cpp" "HAL_SITL_Class.cpp" "I2CDevice.cpp" "RCInput.cpp" "RCOutput.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AnalogIn.cpp"
    "CANSocketIface.cpp"
    "CAN_Multicast.cpp"
    "CAN_SocketCAN.cpp"
    "DSP.cpp"
    "GPIO.cpp"
    "HAL_SITL_Class.cpp"
    "I2CDevice.cpp"
    "RCInput.cpp"
    "RCOutput.cpp"
    "Scheduler.cpp"
    "Semaphores.cpp"
    "sitl_airspeed.cpp"
    "SITL_cmdline.cpp"
    "SITL_Periph_State.cpp"
    "sitl_rangefinder.cpp"
    "SITL_State.cpp"
    "SITL_State_common.cpp"
    "SPIDevice.cpp"
    "Storage.cpp"
    "system.cpp"
    "ToneAlarm_SF.cpp"
    "UARTDriver.cpp"
    "UART_utils.cpp"
    "Util.cpp"
    "AnalogIn.cpp"
    "CANSocketIface.cpp"
    "CAN_Multicast.cpp"
    "CAN_SocketCAN.cpp"
    "DSP.cpp"
    "GPIO.cpp"
    "HAL_SITL_Class.cpp"
    "I2CDevice.cpp"
    "RCInput.cpp"
    "RCOutput.cpp"
    "Scheduler.cpp"
    "Semaphores.cpp"
    "sitl_airspeed.cpp"
    "SITL_cmdline.cpp"
    "SITL_Periph_State.cpp"
    "sitl_rangefinder.cpp"
    "SITL_State.cpp"
    "SITL_State_common.cpp"
    "SPIDevice.cpp"
    "Storage.cpp"
    "system.cpp"
    "ToneAlarm_SF.cpp"
    "UARTDriver.cpp"
    "UART_utils.cpp"
    "Util.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
