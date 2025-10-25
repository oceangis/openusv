# ArduPilot AP_AHRS 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_AHRS.cpp" "AP_AHRS_Backend.cpp" "AP_AHRS_DCM.cpp" "AP_AHRS_External.cpp" "AP_AHRS_Logging.cpp" "AP_AHRS_SIM.cpp" "AP_AHRS_View.cpp" "AP_AHRS.cpp" "AP_AHRS_Backend.cpp" "AP_AHRS_DCM.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_AHRS.cpp"
    "AP_AHRS_Backend.cpp"
    "AP_AHRS_DCM.cpp"
    "AP_AHRS_External.cpp"
    "AP_AHRS_Logging.cpp"
    "AP_AHRS_SIM.cpp"
    "AP_AHRS_View.cpp"
    "AP_AHRS.cpp"
    "AP_AHRS_Backend.cpp"
    "AP_AHRS_DCM.cpp"
    "AP_AHRS_External.cpp"
    "AP_AHRS_Logging.cpp"
    "AP_AHRS_SIM.cpp"
    "AP_AHRS_View.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
