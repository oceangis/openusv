# ArduPilot AP_NavEKF2 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_NavEKF2.cpp" "AP_NavEKF2_AirDataFusion.cpp" "AP_NavEKF2_Control.cpp" "AP_NavEKF2_core.cpp" "AP_NavEKF2_Logging.cpp" "AP_NavEKF2_MagFusion.cpp" "AP_NavEKF2_Measurements.cpp" "AP_NavEKF2_OptFlowFusion.cpp" "AP_NavEKF2_Outputs.cpp" "AP_NavEKF2_PosVelFusion.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_NavEKF2.cpp"
    "AP_NavEKF2_AirDataFusion.cpp"
    "AP_NavEKF2_Control.cpp"
    "AP_NavEKF2_core.cpp"
    "AP_NavEKF2_Logging.cpp"
    "AP_NavEKF2_MagFusion.cpp"
    "AP_NavEKF2_Measurements.cpp"
    "AP_NavEKF2_OptFlowFusion.cpp"
    "AP_NavEKF2_Outputs.cpp"
    "AP_NavEKF2_PosVelFusion.cpp"
    "AP_NavEKF2_RngBcnFusion.cpp"
    "AP_NavEKF2_VehicleStatus.cpp"
    "AP_NavEKF_GyroBias.cpp"
    "AP_NavEKF2.cpp"
    "AP_NavEKF2_AirDataFusion.cpp"
    "AP_NavEKF2_Control.cpp"
    "AP_NavEKF2_core.cpp"
    "AP_NavEKF2_Logging.cpp"
    "AP_NavEKF2_MagFusion.cpp"
    "AP_NavEKF2_Measurements.cpp"
    "AP_NavEKF2_OptFlowFusion.cpp"
    "AP_NavEKF2_Outputs.cpp"
    "AP_NavEKF2_PosVelFusion.cpp"
    "AP_NavEKF2_RngBcnFusion.cpp"
    "AP_NavEKF2_VehicleStatus.cpp"
    "AP_NavEKF_GyroBias.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
