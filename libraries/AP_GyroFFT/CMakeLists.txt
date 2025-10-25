# ArduPilot AP_GyroFFT 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_GyroFFT.cpp" "AP_GyroFFT.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "CMSIS_5" "CMSIS_5/include" "CMSIS_5/lib"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_GyroFFT.cpp"
    "AP_GyroFFT.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "CMSIS_5"
    "CMSIS_5/include"
    "CMSIS_5/lib"
)
