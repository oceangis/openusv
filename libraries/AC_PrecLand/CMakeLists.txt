# ArduPilot AC_PrecLand 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AC_PrecLand.cpp" "AC_PrecLand_IRLock.cpp" "AC_PrecLand_MAVLink.cpp" "AC_PrecLand_SITL.cpp" "AC_PrecLand_SITL_Gazebo.cpp" "AC_PrecLand_StateMachine.cpp" "PosVelEKF.cpp" "AC_PrecLand.cpp" "AC_PrecLand_IRLock.cpp" "AC_PrecLand_MAVLink.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AC_PrecLand.cpp"
    "AC_PrecLand_IRLock.cpp"
    "AC_PrecLand_MAVLink.cpp"
    "AC_PrecLand_SITL.cpp"
    "AC_PrecLand_SITL_Gazebo.cpp"
    "AC_PrecLand_StateMachine.cpp"
    "PosVelEKF.cpp"
    "AC_PrecLand.cpp"
    "AC_PrecLand_IRLock.cpp"
    "AC_PrecLand_MAVLink.cpp"
    "AC_PrecLand_SITL.cpp"
    "AC_PrecLand_SITL_Gazebo.cpp"
    "AC_PrecLand_StateMachine.cpp"
    "PosVelEKF.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
