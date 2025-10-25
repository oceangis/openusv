# ArduPilot AP_Soaring 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Soaring.cpp" "ExtendedKalmanFilter.cpp" "SpeedToFly.cpp" "Variometer.cpp" "AP_Soaring.cpp" "ExtendedKalmanFilter.cpp" "SpeedToFly.cpp" "Variometer.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Soaring.cpp"
    "ExtendedKalmanFilter.cpp"
    "SpeedToFly.cpp"
    "Variometer.cpp"
    "AP_Soaring.cpp"
    "ExtendedKalmanFilter.cpp"
    "SpeedToFly.cpp"
    "Variometer.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
