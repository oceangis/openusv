# ArduPilot AP_DroneCAN 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Canard_iface.cpp" "AP_DroneCAN.cpp" "AP_DroneCAN_DNA_Server.cpp" "AP_DroneCAN_serial.cpp" "AP_Canard_iface.cpp" "AP_DroneCAN.cpp" "AP_DroneCAN_DNA_Server.cpp" "AP_DroneCAN_serial.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "canard" "dsdl"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Canard_iface.cpp"
    "AP_DroneCAN.cpp"
    "AP_DroneCAN_DNA_Server.cpp"
    "AP_DroneCAN_serial.cpp"
    "AP_Canard_iface.cpp"
    "AP_DroneCAN.cpp"
    "AP_DroneCAN_DNA_Server.cpp"
    "AP_DroneCAN_serial.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "canard"
    "dsdl"
)
