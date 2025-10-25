# ArduPilot AP_Radio 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Radio.cpp" "AP_Radio_backend.cpp" "AP_Radio_bk2425.cpp" "AP_Radio_cc2500.cpp" "AP_Radio_cypress.cpp" "driver_bk2425.cpp" "driver_cc2500.cpp" "AP_Radio.cpp" "AP_Radio_backend.cpp" "AP_Radio_bk2425.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Radio.cpp"
    "AP_Radio_backend.cpp"
    "AP_Radio_bk2425.cpp"
    "AP_Radio_cc2500.cpp"
    "AP_Radio_cypress.cpp"
    "driver_bk2425.cpp"
    "driver_cc2500.cpp"
    "AP_Radio.cpp"
    "AP_Radio_backend.cpp"
    "AP_Radio_bk2425.cpp"
    "AP_Radio_cc2500.cpp"
    "AP_Radio_cypress.cpp"
    "driver_bk2425.cpp"
    "driver_cc2500.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
