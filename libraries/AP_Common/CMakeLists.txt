# ArduPilot AP_Common 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Common.cpp" "AP_ExpandingArray.cpp" "AP_FWVersion.cpp" "c++.cpp" "ExpandingString.cpp" "float16.cpp" "Location.cpp" "NMEA.cpp" "sorting.cpp" "time.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "missing" "missing/sys"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Common.cpp"
    "AP_ExpandingArray.cpp"
    "AP_FWVersion.cpp"
    "c++.cpp"
    "ExpandingString.cpp"
    "float16.cpp"
    "Location.cpp"
    "NMEA.cpp"
    "sorting.cpp"
    "time.cpp"
    "AP_Common.cpp"
    "AP_ExpandingArray.cpp"
    "AP_FWVersion.cpp"
    "c++.cpp"
    "ExpandingString.cpp"
    "float16.cpp"
    "Location.cpp"
    "NMEA.cpp"
    "sorting.cpp"
    "time.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "missing"
    "missing/sys"
)
