# ArduPilot AP_GPS 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_GPS.cpp" "AP_GPS_Blended.cpp" "AP_GPS_DroneCAN.cpp" "AP_GPS_ERB.cpp" "AP_GPS_ExternalAHRS.cpp" "AP_GPS_GSOF.cpp" "AP_GPS_MAV.cpp" "AP_GPS_MSP.cpp" "AP_GPS_NMEA.cpp" "AP_GPS_NOVA.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_GPS.cpp"
    "AP_GPS_Blended.cpp"
    "AP_GPS_DroneCAN.cpp"
    "AP_GPS_ERB.cpp"
    "AP_GPS_ExternalAHRS.cpp"
    "AP_GPS_GSOF.cpp"
    "AP_GPS_MAV.cpp"
    "AP_GPS_MSP.cpp"
    "AP_GPS_NMEA.cpp"
    "AP_GPS_NOVA.cpp"
    "AP_GPS_Params.cpp"
    "AP_GPS_SBF.cpp"
    "AP_GPS_SBP.cpp"
    "AP_GPS_SBP2.cpp"
    "AP_GPS_SIRF.cpp"
    "AP_GPS_SITL.cpp"
    "AP_GPS_UBLOX.cpp"
    "GPS_Backend.cpp"
    "MovingBase.cpp"
    "RTCM3_Parser.cpp"
    "AP_GPS.cpp"
    "AP_GPS_Blended.cpp"
    "AP_GPS_DroneCAN.cpp"
    "AP_GPS_ERB.cpp"
    "AP_GPS_ExternalAHRS.cpp"
    "AP_GPS_GSOF.cpp"
    "AP_GPS_MAV.cpp"
    "AP_GPS_MSP.cpp"
    "AP_GPS_NMEA.cpp"
    "AP_GPS_NOVA.cpp"
    "AP_GPS_Params.cpp"
    "AP_GPS_SBF.cpp"
    "AP_GPS_SBP.cpp"
    "AP_GPS_SBP2.cpp"
    "AP_GPS_SIRF.cpp"
    "AP_GPS_SITL.cpp"
    "AP_GPS_UBLOX.cpp"
    "GPS_Backend.cpp"
    "MovingBase.cpp"
    "RTCM3_Parser.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
