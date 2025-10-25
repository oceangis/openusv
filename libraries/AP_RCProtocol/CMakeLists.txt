# ArduPilot AP_RCProtocol 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_RCProtocol.cpp" "AP_RCProtocol_Backend.cpp" "AP_RCProtocol_CRSF.cpp" "AP_RCProtocol_DroneCAN.cpp" "AP_RCProtocol_DSM.cpp" "AP_RCProtocol_Emlid_RCIO.cpp" "AP_RCProtocol_FDM.cpp" "AP_RCProtocol_FPort.cpp" "AP_RCProtocol_FPort2.cpp" "AP_RCProtocol_GHST.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_RCProtocol.cpp"
    "AP_RCProtocol_Backend.cpp"
    "AP_RCProtocol_CRSF.cpp"
    "AP_RCProtocol_DroneCAN.cpp"
    "AP_RCProtocol_DSM.cpp"
    "AP_RCProtocol_Emlid_RCIO.cpp"
    "AP_RCProtocol_FDM.cpp"
    "AP_RCProtocol_FPort.cpp"
    "AP_RCProtocol_FPort2.cpp"
    "AP_RCProtocol_GHST.cpp"
    "AP_RCProtocol_IBUS.cpp"
    "AP_RCProtocol_IOMCU.cpp"
    "AP_RCProtocol_Joystick_SFML.cpp"
    "AP_RCProtocol_MAVLinkRadio.cpp"
    "AP_RCProtocol_PPMSum.cpp"
    "AP_RCProtocol_Radio.cpp"
    "AP_RCProtocol_SBUS.cpp"
    "AP_RCProtocol_SRXL.cpp"
    "AP_RCProtocol_SRXL2.cpp"
    "AP_RCProtocol_ST24.cpp"
    "AP_RCProtocol_SUMD.cpp"
    "AP_RCProtocol_UDP.cpp"
    "SoftSerial.cpp"
    "spm_srxl.cpp"
    "AP_RCProtocol.cpp"
    "AP_RCProtocol_Backend.cpp"
    "AP_RCProtocol_CRSF.cpp"
    "AP_RCProtocol_DroneCAN.cpp"
    "AP_RCProtocol_DSM.cpp"
    "AP_RCProtocol_Emlid_RCIO.cpp"
    "AP_RCProtocol_FDM.cpp"
    "AP_RCProtocol_FPort.cpp"
    "AP_RCProtocol_FPort2.cpp"
    "AP_RCProtocol_GHST.cpp"
    "AP_RCProtocol_IBUS.cpp"
    "AP_RCProtocol_IOMCU.cpp"
    "AP_RCProtocol_Joystick_SFML.cpp"
    "AP_RCProtocol_MAVLinkRadio.cpp"
    "AP_RCProtocol_PPMSum.cpp"
    "AP_RCProtocol_Radio.cpp"
    "AP_RCProtocol_SBUS.cpp"
    "AP_RCProtocol_SRXL.cpp"
    "AP_RCProtocol_SRXL2.cpp"
    "AP_RCProtocol_ST24.cpp"
    "AP_RCProtocol_SUMD.cpp"
    "AP_RCProtocol_UDP.cpp"
    "SoftSerial.cpp"
    "spm_srxl.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
