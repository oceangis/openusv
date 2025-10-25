# ArduPilot AP_MSP 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_MSP.cpp" "AP_MSP_Telem_Backend.cpp" "AP_MSP_Telem_DisplayPort.cpp" "AP_MSP_Telem_DJI.cpp" "AP_MSP_Telem_Generic.cpp" "msp.cpp" "msp_sbuf.cpp" "AP_MSP.cpp" "AP_MSP_Telem_Backend.cpp" "AP_MSP_Telem_DisplayPort.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "Tools"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_MSP.cpp"
    "AP_MSP_Telem_Backend.cpp"
    "AP_MSP_Telem_DisplayPort.cpp"
    "AP_MSP_Telem_DJI.cpp"
    "AP_MSP_Telem_Generic.cpp"
    "msp.cpp"
    "msp_sbuf.cpp"
    "AP_MSP.cpp"
    "AP_MSP_Telem_Backend.cpp"
    "AP_MSP_Telem_DisplayPort.cpp"
    "AP_MSP_Telem_DJI.cpp"
    "AP_MSP_Telem_Generic.cpp"
    "msp.cpp"
    "msp_sbuf.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "Tools"
)
