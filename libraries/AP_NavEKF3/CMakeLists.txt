# ArduPilot AP_NavEKF3 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_NavEKF3.cpp" "AP_NavEKF3_AirDataFusion.cpp" "AP_NavEKF3_Control.cpp" "AP_NavEKF3_core.cpp" "AP_NavEKF3_GyroBias.cpp" "AP_NavEKF3_Logging.cpp" "AP_NavEKF3_MagFusion.cpp" "AP_NavEKF3_Measurements.cpp" "AP_NavEKF3_OptFlowFusion.cpp" "AP_NavEKF3_Outputs.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "derivation"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_NavEKF3.cpp"
    "AP_NavEKF3_AirDataFusion.cpp"
    "AP_NavEKF3_Control.cpp"
    "AP_NavEKF3_core.cpp"
    "AP_NavEKF3_GyroBias.cpp"
    "AP_NavEKF3_Logging.cpp"
    "AP_NavEKF3_MagFusion.cpp"
    "AP_NavEKF3_Measurements.cpp"
    "AP_NavEKF3_OptFlowFusion.cpp"
    "AP_NavEKF3_Outputs.cpp"
    "AP_NavEKF3_PosVelFusion.cpp"
    "AP_NavEKF3_RngBcnFusion.cpp"
    "AP_NavEKF3_VehicleStatus.cpp"
    "AP_NavEKF3.cpp"
    "AP_NavEKF3_AirDataFusion.cpp"
    "AP_NavEKF3_Control.cpp"
    "AP_NavEKF3_core.cpp"
    "AP_NavEKF3_GyroBias.cpp"
    "AP_NavEKF3_Logging.cpp"
    "AP_NavEKF3_MagFusion.cpp"
    "AP_NavEKF3_Measurements.cpp"
    "AP_NavEKF3_OptFlowFusion.cpp"
    "AP_NavEKF3_Outputs.cpp"
    "AP_NavEKF3_PosVelFusion.cpp"
    "AP_NavEKF3_RngBcnFusion.cpp"
    "AP_NavEKF3_VehicleStatus.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "derivation"
)
