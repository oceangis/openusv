# ArduPilot AP_OSD 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_OSD.cpp" "AP_OSD_Backend.cpp" "AP_OSD_MAX7456.cpp" "AP_OSD_MSP.cpp" "AP_OSD_MSP_DisplayPort.cpp" "AP_OSD_ParamScreen.cpp" "AP_OSD_ParamSetting.cpp" "AP_OSD_Screen.cpp" "AP_OSD_Setting.cpp" "AP_OSD_SITL.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "fonts" "fonts/HDFonts" "fonts/HDFonts/WalkSnail" "fonts/HDFonts/WalkSnail/userfont"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_OSD.cpp"
    "AP_OSD_Backend.cpp"
    "AP_OSD_MAX7456.cpp"
    "AP_OSD_MSP.cpp"
    "AP_OSD_MSP_DisplayPort.cpp"
    "AP_OSD_ParamScreen.cpp"
    "AP_OSD_ParamSetting.cpp"
    "AP_OSD_Screen.cpp"
    "AP_OSD_Setting.cpp"
    "AP_OSD_SITL.cpp"
    "AP_OSD.cpp"
    "AP_OSD_Backend.cpp"
    "AP_OSD_MAX7456.cpp"
    "AP_OSD_MSP.cpp"
    "AP_OSD_MSP_DisplayPort.cpp"
    "AP_OSD_ParamScreen.cpp"
    "AP_OSD_ParamSetting.cpp"
    "AP_OSD_Screen.cpp"
    "AP_OSD_Setting.cpp"
    "AP_OSD_SITL.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "fonts"
    "fonts/HDFonts"
    "fonts/HDFonts/WalkSnail"
    "fonts/HDFonts/WalkSnail/userfont"
    "fonts/HDFonts/WTFOS-DJI"
    "fonts/HDFonts/WTFOS-DJI/Pre-Ver0.12"
    "fonts/HDFonts/WTFOS-DJI/Pre-Ver0.12/AllWhite"
    "fonts/HDFonts/WTFOS-DJI/Pre-Ver0.12/Color"
    "fonts/HDFonts/WTFOS-DJI/Ver0.12_or_later"
)
