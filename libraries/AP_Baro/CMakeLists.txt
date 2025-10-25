# ArduPilot AP_Baro 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Baro.cpp" "AP_Baro_atmosphere.cpp" "AP_Baro_AUAV.cpp" "AP_Baro_Backend.cpp" "AP_Baro_BMP085.cpp" "AP_Baro_BMP280.cpp" "AP_Baro_BMP388.cpp" "AP_Baro_BMP581.cpp" "AP_Baro_DPS280.cpp" "AP_Baro_DroneCAN.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Baro.cpp"
    "AP_Baro_atmosphere.cpp"
    "AP_Baro_AUAV.cpp"
    "AP_Baro_Backend.cpp"
    "AP_Baro_BMP085.cpp"
    "AP_Baro_BMP280.cpp"
    "AP_Baro_BMP388.cpp"
    "AP_Baro_BMP581.cpp"
    "AP_Baro_DPS280.cpp"
    "AP_Baro_DroneCAN.cpp"
    "AP_Baro_Dummy.cpp"
    "AP_Baro_ExternalAHRS.cpp"
    "AP_Baro_FBM320.cpp"
    "AP_Baro_HIL.cpp"
    "AP_Baro_ICM20789.cpp"
    "AP_Baro_ICP101XX.cpp"
    "AP_Baro_ICP201XX.cpp"
    "AP_Baro_KellerLD.cpp"
    "AP_Baro_Logging.cpp"
    "AP_Baro_LPS2XH.cpp"
    "AP_Baro_MS5611.cpp"
    "AP_Baro_MSP.cpp"
    "AP_Baro_SITL.cpp"
    "AP_Baro_SPL06.cpp"
    "AP_Baro_Wind.cpp"
    "AP_Baro.cpp"
    "AP_Baro_atmosphere.cpp"
    "AP_Baro_AUAV.cpp"
    "AP_Baro_Backend.cpp"
    "AP_Baro_BMP085.cpp"
    "AP_Baro_BMP280.cpp"
    "AP_Baro_BMP388.cpp"
    "AP_Baro_BMP581.cpp"
    "AP_Baro_DPS280.cpp"
    "AP_Baro_DroneCAN.cpp"
    "AP_Baro_Dummy.cpp"
    "AP_Baro_ExternalAHRS.cpp"
    "AP_Baro_FBM320.cpp"
    "AP_Baro_HIL.cpp"
    "AP_Baro_ICM20789.cpp"
    "AP_Baro_ICP101XX.cpp"
    "AP_Baro_ICP201XX.cpp"
    "AP_Baro_KellerLD.cpp"
    "AP_Baro_Logging.cpp"
    "AP_Baro_LPS2XH.cpp"
    "AP_Baro_MS5611.cpp"
    "AP_Baro_MSP.cpp"
    "AP_Baro_SITL.cpp"
    "AP_Baro_SPL06.cpp"
    "AP_Baro_Wind.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
