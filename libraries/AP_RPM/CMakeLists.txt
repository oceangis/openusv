# ArduPilot AP_RPM 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_RPM.cpp" "AP_RPM_Params.cpp" "RPM_Backend.cpp" "RPM_DroneCAN.cpp" "RPM_EFI.cpp" "RPM_ESC_Telem.cpp" "RPM_Generator.cpp" "RPM_HarmonicNotch.cpp" "RPM_Pin.cpp" "RPM_SITL.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_RPM.cpp"
    "AP_RPM_Params.cpp"
    "RPM_Backend.cpp"
    "RPM_DroneCAN.cpp"
    "RPM_EFI.cpp"
    "RPM_ESC_Telem.cpp"
    "RPM_Generator.cpp"
    "RPM_HarmonicNotch.cpp"
    "RPM_Pin.cpp"
    "RPM_SITL.cpp"
    "AP_RPM.cpp"
    "AP_RPM_Params.cpp"
    "RPM_Backend.cpp"
    "RPM_DroneCAN.cpp"
    "RPM_EFI.cpp"
    "RPM_ESC_Telem.cpp"
    "RPM_Generator.cpp"
    "RPM_HarmonicNotch.cpp"
    "RPM_Pin.cpp"
    "RPM_SITL.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
