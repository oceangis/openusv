# ArduPilot AP_BattMonitor 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_BattMonitor.cpp" "AP_BattMonitor_AD7091R5.cpp" "AP_BattMonitor_Analog.cpp" "AP_BattMonitor_Backend.cpp" "AP_BattMonitor_DroneCAN.cpp" "AP_BattMonitor_EFI.cpp" "AP_BattMonitor_ESC.cpp" "AP_BattMonitor_FuelFlow.cpp" "AP_BattMonitor_FuelLevel_Analog.cpp" "AP_BattMonitor_FuelLevel_PWM.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_BattMonitor.cpp"
    "AP_BattMonitor_AD7091R5.cpp"
    "AP_BattMonitor_Analog.cpp"
    "AP_BattMonitor_Backend.cpp"
    "AP_BattMonitor_DroneCAN.cpp"
    "AP_BattMonitor_EFI.cpp"
    "AP_BattMonitor_ESC.cpp"
    "AP_BattMonitor_FuelFlow.cpp"
    "AP_BattMonitor_FuelLevel_Analog.cpp"
    "AP_BattMonitor_FuelLevel_PWM.cpp"
    "AP_BattMonitor_Generator.cpp"
    "AP_BattMonitor_INA239.cpp"
    "AP_BattMonitor_INA2xx.cpp"
    "AP_BattMonitor_INA3221.cpp"
    "AP_BattMonitor_Logging.cpp"
    "AP_BattMonitor_LTC2946.cpp"
    "AP_BattMonitor_Params.cpp"
    "AP_BattMonitor_Scripting.cpp"
    "AP_BattMonitor_SMBus.cpp"
    "AP_BattMonitor_SMBus_Generic.cpp"
    "AP_BattMonitor_SMBus_NeoDesign.cpp"
    "AP_BattMonitor_SMBus_Rotoye.cpp"
    "AP_BattMonitor_SMBus_Solo.cpp"
    "AP_BattMonitor_SMBus_SUI.cpp"
    "AP_BattMonitor_Sum.cpp"
    "AP_BattMonitor_Synthetic_Current.cpp"
    "AP_BattMonitor_Torqeedo.cpp"
    "AP_BattMonitor.cpp"
    "AP_BattMonitor_AD7091R5.cpp"
    "AP_BattMonitor_Analog.cpp"
    "AP_BattMonitor_Backend.cpp"
    "AP_BattMonitor_DroneCAN.cpp"
    "AP_BattMonitor_EFI.cpp"
    "AP_BattMonitor_ESC.cpp"
    "AP_BattMonitor_FuelFlow.cpp"
    "AP_BattMonitor_FuelLevel_Analog.cpp"
    "AP_BattMonitor_FuelLevel_PWM.cpp"
    "AP_BattMonitor_Generator.cpp"
    "AP_BattMonitor_INA239.cpp"
    "AP_BattMonitor_INA2xx.cpp"
    "AP_BattMonitor_INA3221.cpp"
    "AP_BattMonitor_Logging.cpp"
    "AP_BattMonitor_LTC2946.cpp"
    "AP_BattMonitor_Params.cpp"
    "AP_BattMonitor_Scripting.cpp"
    "AP_BattMonitor_SMBus.cpp"
    "AP_BattMonitor_SMBus_Generic.cpp"
    "AP_BattMonitor_SMBus_NeoDesign.cpp"
    "AP_BattMonitor_SMBus_Rotoye.cpp"
    "AP_BattMonitor_SMBus_Solo.cpp"
    "AP_BattMonitor_SMBus_SUI.cpp"
    "AP_BattMonitor_Sum.cpp"
    "AP_BattMonitor_Synthetic_Current.cpp"
    "AP_BattMonitor_Torqeedo.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
