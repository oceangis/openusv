# ArduPilot AP_ESC_Telem 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_ESC_Telem.cpp" "AP_ESC_Telem_Backend.cpp" "AP_ESC_Telem_SITL.cpp" "AP_ESC_Telem.cpp" "AP_ESC_Telem_Backend.cpp" "AP_ESC_Telem_SITL.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_ESC_Telem.cpp"
    "AP_ESC_Telem_Backend.cpp"
    "AP_ESC_Telem_SITL.cpp"
    "AP_ESC_Telem.cpp"
    "AP_ESC_Telem_Backend.cpp"
    "AP_ESC_Telem_SITL.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
