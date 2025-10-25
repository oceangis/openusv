# ArduPilot AP_Generator 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Generator.cpp" "AP_Generator_Backend.cpp" "AP_Generator_IE_2400.cpp" "AP_Generator_IE_650_800.cpp" "AP_Generator_IE_FuelCell.cpp" "AP_Generator_Loweheiser.cpp" "AP_Generator_RichenPower.cpp" "AP_Generator.cpp" "AP_Generator_Backend.cpp" "AP_Generator_IE_2400.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "scripts"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Generator.cpp"
    "AP_Generator_Backend.cpp"
    "AP_Generator_IE_2400.cpp"
    "AP_Generator_IE_650_800.cpp"
    "AP_Generator_IE_FuelCell.cpp"
    "AP_Generator_Loweheiser.cpp"
    "AP_Generator_RichenPower.cpp"
    "AP_Generator.cpp"
    "AP_Generator_Backend.cpp"
    "AP_Generator_IE_2400.cpp"
    "AP_Generator_IE_650_800.cpp"
    "AP_Generator_IE_FuelCell.cpp"
    "AP_Generator_Loweheiser.cpp"
    "AP_Generator_RichenPower.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "scripts"
)
