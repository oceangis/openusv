# ArduPilot AP_DAC 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_DAC.cpp" "AP_DAC_MCP40D1x.cpp" "AP_DAC_Params.cpp" "AP_DAC_TIx3204.cpp" "AP_DAC.cpp" "AP_DAC_MCP40D1x.cpp" "AP_DAC_Params.cpp" "AP_DAC_TIx3204.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_DAC.cpp"
    "AP_DAC_MCP40D1x.cpp"
    "AP_DAC_Params.cpp"
    "AP_DAC_TIx3204.cpp"
    "AP_DAC.cpp"
    "AP_DAC_MCP40D1x.cpp"
    "AP_DAC_Params.cpp"
    "AP_DAC_TIx3204.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
