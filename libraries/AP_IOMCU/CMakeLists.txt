# ArduPilot AP_IOMCU 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_IOMCU.cpp" "fw_uploader.cpp" "AP_IOMCU.cpp" "fw_uploader.cpp" "iofirmware/analog.cpp" "iofirmware/iofirmware.cpp" "iofirmware/mixer.cpp" "iofirmware/rc.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "iofirmware"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_IOMCU.cpp"
    "fw_uploader.cpp"
    "AP_IOMCU.cpp"
    "fw_uploader.cpp"
    "iofirmware/analog.cpp"
    "iofirmware/iofirmware.cpp"
    "iofirmware/mixer.cpp"
    "iofirmware/rc.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "iofirmware"
)
