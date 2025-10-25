# ArduPilot AP_Networking 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Networking.cpp" "AP_Networking_address.cpp" "AP_Networking_Backend.cpp" "AP_Networking_CAN.cpp" "AP_Networking_macaddr.cpp" "AP_Networking_port.cpp" "AP_Networking_PPP.cpp" "AP_Networking_tests.cpp" "AP_Networking.cpp" "AP_Networking_address.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "config" "lwip_hal" "lwip_hal/arch" "lwip_hal/include"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Networking.cpp"
    "AP_Networking_address.cpp"
    "AP_Networking_Backend.cpp"
    "AP_Networking_CAN.cpp"
    "AP_Networking_macaddr.cpp"
    "AP_Networking_port.cpp"
    "AP_Networking_PPP.cpp"
    "AP_Networking_tests.cpp"
    "AP_Networking.cpp"
    "AP_Networking_address.cpp"
    "AP_Networking_Backend.cpp"
    "AP_Networking_CAN.cpp"
    "AP_Networking_macaddr.cpp"
    "AP_Networking_port.cpp"
    "AP_Networking_PPP.cpp"
    "AP_Networking_tests.cpp"
    "lwip_hal/arch/sys_arch.cpp"
    "lwip_hal/arch/evtimer.c"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "config"
    "lwip_hal"
    "lwip_hal/arch"
    "lwip_hal/include"
    "lwip_hal/include/arch"
)
