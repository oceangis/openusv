# ArduPilot AC_CustomControl 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AC_CustomControl.cpp" "AC_CustomControl_Empty.cpp" "AC_CustomControl_PID.cpp" "AC_CustomControl.cpp" "AC_CustomControl_Empty.cpp" "AC_CustomControl_PID.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AC_CustomControl.cpp"
    "AC_CustomControl_Empty.cpp"
    "AC_CustomControl_PID.cpp"
    "AC_CustomControl.cpp"
    "AC_CustomControl_Empty.cpp"
    "AC_CustomControl_PID.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
