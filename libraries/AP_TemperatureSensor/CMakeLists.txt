# ArduPilot AP_TemperatureSensor 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_TemperatureSensor.cpp" "AP_TemperatureSensor_Analog.cpp" "AP_TemperatureSensor_Backend.cpp" "AP_TemperatureSensor_DroneCAN.cpp" "AP_TemperatureSensor_MAX31865.cpp" "AP_TemperatureSensor_MCP9600.cpp" "AP_TemperatureSensor_MLX90614.cpp" "AP_TemperatureSensor_Params.cpp" "AP_TemperatureSensor_SHT3x.cpp" "AP_TemperatureSensor_TSYS01.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_TemperatureSensor.cpp"
    "AP_TemperatureSensor_Analog.cpp"
    "AP_TemperatureSensor_Backend.cpp"
    "AP_TemperatureSensor_DroneCAN.cpp"
    "AP_TemperatureSensor_MAX31865.cpp"
    "AP_TemperatureSensor_MCP9600.cpp"
    "AP_TemperatureSensor_MLX90614.cpp"
    "AP_TemperatureSensor_Params.cpp"
    "AP_TemperatureSensor_SHT3x.cpp"
    "AP_TemperatureSensor_TSYS01.cpp"
    "AP_TemperatureSensor_TSYS03.cpp"
    "AP_TemperatureSensor.cpp"
    "AP_TemperatureSensor_Analog.cpp"
    "AP_TemperatureSensor_Backend.cpp"
    "AP_TemperatureSensor_DroneCAN.cpp"
    "AP_TemperatureSensor_MAX31865.cpp"
    "AP_TemperatureSensor_MCP9600.cpp"
    "AP_TemperatureSensor_MLX90614.cpp"
    "AP_TemperatureSensor_Params.cpp"
    "AP_TemperatureSensor_SHT3x.cpp"
    "AP_TemperatureSensor_TSYS01.cpp"
    "AP_TemperatureSensor_TSYS03.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
