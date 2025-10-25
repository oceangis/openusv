# ArduPilot AP_DAL 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_DAL.cpp" "AP_DAL_Airspeed.cpp" "AP_DAL_Baro.cpp" "AP_DAL_Beacon.cpp" "AP_DAL_Compass.cpp" "AP_DAL_GPS.cpp" "AP_DAL_InertialSensor.cpp" "AP_DAL_RangeFinder.cpp" "AP_DAL_VisualOdom.cpp" "AP_DAL.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_DAL.cpp"
    "AP_DAL_Airspeed.cpp"
    "AP_DAL_Baro.cpp"
    "AP_DAL_Beacon.cpp"
    "AP_DAL_Compass.cpp"
    "AP_DAL_GPS.cpp"
    "AP_DAL_InertialSensor.cpp"
    "AP_DAL_RangeFinder.cpp"
    "AP_DAL_VisualOdom.cpp"
    "AP_DAL.cpp"
    "AP_DAL_Airspeed.cpp"
    "AP_DAL_Baro.cpp"
    "AP_DAL_Beacon.cpp"
    "AP_DAL_Compass.cpp"
    "AP_DAL_GPS.cpp"
    "AP_DAL_InertialSensor.cpp"
    "AP_DAL_RangeFinder.cpp"
    "AP_DAL_VisualOdom.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
