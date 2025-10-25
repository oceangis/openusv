# ArduPilot AP_WindVane 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_WindVane.cpp" "AP_WindVane_Airspeed.cpp" "AP_WindVane_Analog.cpp" "AP_WindVane_Backend.cpp" "AP_WindVane_Home.cpp" "AP_WindVane_ModernDevice.cpp" "AP_WindVane_NMEA.cpp" "AP_WindVane_RPM.cpp" "AP_WindVane_SITL.cpp" "AP_WindVane.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "Tools" "Tools/Bluetooth NMEA receiver"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_WindVane.cpp"
    "AP_WindVane_Airspeed.cpp"
    "AP_WindVane_Analog.cpp"
    "AP_WindVane_Backend.cpp"
    "AP_WindVane_Home.cpp"
    "AP_WindVane_ModernDevice.cpp"
    "AP_WindVane_NMEA.cpp"
    "AP_WindVane_RPM.cpp"
    "AP_WindVane_SITL.cpp"
    "AP_WindVane.cpp"
    "AP_WindVane_Airspeed.cpp"
    "AP_WindVane_Analog.cpp"
    "AP_WindVane_Backend.cpp"
    "AP_WindVane_Home.cpp"
    "AP_WindVane_ModernDevice.cpp"
    "AP_WindVane_NMEA.cpp"
    "AP_WindVane_RPM.cpp"
    "AP_WindVane_SITL.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "Tools"
    "Tools/Bluetooth NMEA receiver"
)
