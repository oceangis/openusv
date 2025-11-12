# ArduPilot AP_Param 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Param.cpp" "AP_ParamT.cpp"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Param.cpp"
    "AP_ParamT.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
