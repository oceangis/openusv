# ArduPilot AP_Beacon 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Beacon.cpp" "AP_Beacon_Backend.cpp" "AP_Beacon_Marvelmind.cpp" "AP_Beacon_Nooploop.cpp" "AP_Beacon_Pozyx.cpp" "AP_Beacon_SITL.cpp" "AP_Beacon.cpp" "AP_Beacon_Backend.cpp" "AP_Beacon_Marvelmind.cpp" "AP_Beacon_Nooploop.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "sitl"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Beacon.cpp"
    "AP_Beacon_Backend.cpp"
    "AP_Beacon_Marvelmind.cpp"
    "AP_Beacon_Nooploop.cpp"
    "AP_Beacon_Pozyx.cpp"
    "AP_Beacon_SITL.cpp"
    "AP_Beacon.cpp"
    "AP_Beacon_Backend.cpp"
    "AP_Beacon_Marvelmind.cpp"
    "AP_Beacon_Nooploop.cpp"
    "AP_Beacon_Pozyx.cpp"
    "AP_Beacon_SITL.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "sitl"
)
