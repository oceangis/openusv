# ArduPilot AC_AttitudeControl 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AC_AttitudeControl.cpp" "AC_AttitudeControl_Heli.cpp" "AC_AttitudeControl_Logging.cpp" "AC_AttitudeControl_Multi.cpp" "AC_AttitudeControl_Multi_6DoF.cpp" "AC_AttitudeControl_Sub.cpp" "AC_AttitudeControl_TS.cpp" "AC_CommandModel.cpp" "AC_PosControl.cpp" "AC_PosControl_Logging.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AC_AttitudeControl.cpp"
    "AC_AttitudeControl_Heli.cpp"
    "AC_AttitudeControl_Logging.cpp"
    "AC_AttitudeControl_Multi.cpp"
    "AC_AttitudeControl_Multi_6DoF.cpp"
    "AC_AttitudeControl_Sub.cpp"
    "AC_AttitudeControl_TS.cpp"
    "AC_CommandModel.cpp"
    "AC_PosControl.cpp"
    "AC_PosControl_Logging.cpp"
    "AC_WeatherVane.cpp"
    "AC_AttitudeControl.cpp"
    "AC_AttitudeControl_Heli.cpp"
    "AC_AttitudeControl_Logging.cpp"
    "AC_AttitudeControl_Multi.cpp"
    "AC_AttitudeControl_Multi_6DoF.cpp"
    "AC_AttitudeControl_Sub.cpp"
    "AC_AttitudeControl_TS.cpp"
    "AC_CommandModel.cpp"
    "AC_PosControl.cpp"
    "AC_PosControl_Logging.cpp"
    "AC_WeatherVane.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
