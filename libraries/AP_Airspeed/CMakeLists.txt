# ArduPilot AP_Airspeed 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "Airspeed_Calibration.cpp" "AP_Airspeed.cpp" "AP_Airspeed_analog.cpp" "AP_Airspeed_ASP5033.cpp" "AP_Airspeed_AUAV.cpp" "AP_Airspeed_Backend.cpp" "AP_Airspeed_DLVR.cpp" "AP_Airspeed_DroneCAN.cpp" "AP_Airspeed_External.cpp" "AP_Airspeed_Health.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "models"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "Airspeed_Calibration.cpp"
    "AP_Airspeed.cpp"
    "AP_Airspeed_analog.cpp"
    "AP_Airspeed_ASP5033.cpp"
    "AP_Airspeed_AUAV.cpp"
    "AP_Airspeed_Backend.cpp"
    "AP_Airspeed_DLVR.cpp"
    "AP_Airspeed_DroneCAN.cpp"
    "AP_Airspeed_External.cpp"
    "AP_Airspeed_Health.cpp"
    "AP_Airspeed_MS4525.cpp"
    "AP_Airspeed_MS5525.cpp"
    "AP_Airspeed_MSP.cpp"
    "AP_Airspeed_NMEA.cpp"
    "AP_Airspeed_Params.cpp"
    "AP_Airspeed_SDP3X.cpp"
    "AP_Airspeed_SITL.cpp"
    "Airspeed_Calibration.cpp"
    "AP_Airspeed.cpp"
    "AP_Airspeed_analog.cpp"
    "AP_Airspeed_ASP5033.cpp"
    "AP_Airspeed_AUAV.cpp"
    "AP_Airspeed_Backend.cpp"
    "AP_Airspeed_DLVR.cpp"
    "AP_Airspeed_DroneCAN.cpp"
    "AP_Airspeed_External.cpp"
    "AP_Airspeed_Health.cpp"
    "AP_Airspeed_MS4525.cpp"
    "AP_Airspeed_MS5525.cpp"
    "AP_Airspeed_MSP.cpp"
    "AP_Airspeed_NMEA.cpp"
    "AP_Airspeed_Params.cpp"
    "AP_Airspeed_SDP3X.cpp"
    "AP_Airspeed_SITL.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "models"
)
