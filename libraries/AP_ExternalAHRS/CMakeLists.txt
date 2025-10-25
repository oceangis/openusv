# ArduPilot AP_ExternalAHRS 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_ExternalAHRS.cpp" "AP_ExternalAHRS_backend.cpp" "AP_ExternalAHRS_InertialLabs.cpp" "AP_ExternalAHRS_MicroStrain5.cpp" "AP_ExternalAHRS_MicroStrain7.cpp" "AP_ExternalAHRS_SBG.cpp" "AP_ExternalAHRS_VectorNav.cpp" "MicroStrain_common.cpp" "AP_ExternalAHRS.cpp" "AP_ExternalAHRS_backend.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_ExternalAHRS.cpp"
    "AP_ExternalAHRS_backend.cpp"
    "AP_ExternalAHRS_InertialLabs.cpp"
    "AP_ExternalAHRS_MicroStrain5.cpp"
    "AP_ExternalAHRS_MicroStrain7.cpp"
    "AP_ExternalAHRS_SBG.cpp"
    "AP_ExternalAHRS_VectorNav.cpp"
    "MicroStrain_common.cpp"
    "AP_ExternalAHRS.cpp"
    "AP_ExternalAHRS_backend.cpp"
    "AP_ExternalAHRS_InertialLabs.cpp"
    "AP_ExternalAHRS_MicroStrain5.cpp"
    "AP_ExternalAHRS_MicroStrain7.cpp"
    "AP_ExternalAHRS_SBG.cpp"
    "AP_ExternalAHRS_VectorNav.cpp"
    "MicroStrain_common.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
