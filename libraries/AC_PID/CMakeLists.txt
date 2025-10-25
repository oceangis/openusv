# ArduPilot AC_PID 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AC_HELI_PID.cpp" "AC_P.cpp" "AC_PI.cpp" "AC_PID.cpp" "AC_PID_2D.cpp" "AC_PID_Basic.cpp" "AC_PI_2D.cpp" "AC_P_1D.cpp" "AC_P_2D.cpp" "AC_HELI_PID.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AC_HELI_PID.cpp"
    "AC_P.cpp"
    "AC_PI.cpp"
    "AC_PID.cpp"
    "AC_PID_2D.cpp"
    "AC_PID_Basic.cpp"
    "AC_PI_2D.cpp"
    "AC_P_1D.cpp"
    "AC_P_2D.cpp"
    "AC_HELI_PID.cpp"
    "AC_P.cpp"
    "AC_PI.cpp"
    "AC_PID.cpp"
    "AC_PID_2D.cpp"
    "AC_PID_Basic.cpp"
    "AC_PI_2D.cpp"
    "AC_P_1D.cpp"
    "AC_P_2D.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
