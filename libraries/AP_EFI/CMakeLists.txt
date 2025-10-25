# ArduPilot AP_EFI 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_EFI.cpp" "AP_EFI_Backend.cpp" "AP_EFI_Currawong_ECU.cpp" "AP_EFI_DroneCAN.cpp" "AP_EFI_Loweheiser.cpp" "AP_EFI_MAV.cpp" "AP_EFI_NWPMU.cpp" "AP_EFI_Scripting.cpp" "AP_EFI_Serial_Hirth.cpp" "AP_EFI_Serial_Lutan.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_EFI.cpp"
    "AP_EFI_Backend.cpp"
    "AP_EFI_Currawong_ECU.cpp"
    "AP_EFI_DroneCAN.cpp"
    "AP_EFI_Loweheiser.cpp"
    "AP_EFI_MAV.cpp"
    "AP_EFI_NWPMU.cpp"
    "AP_EFI_Scripting.cpp"
    "AP_EFI_Serial_Hirth.cpp"
    "AP_EFI_Serial_Lutan.cpp"
    "AP_EFI_Serial_MS.cpp"
    "AP_EFI_ThrottleLinearisation.cpp"
    "AP_EFI.cpp"
    "AP_EFI_Backend.cpp"
    "AP_EFI_Currawong_ECU.cpp"
    "AP_EFI_DroneCAN.cpp"
    "AP_EFI_Loweheiser.cpp"
    "AP_EFI_MAV.cpp"
    "AP_EFI_NWPMU.cpp"
    "AP_EFI_Scripting.cpp"
    "AP_EFI_Serial_Hirth.cpp"
    "AP_EFI_Serial_Lutan.cpp"
    "AP_EFI_Serial_MS.cpp"
    "AP_EFI_ThrottleLinearisation.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
