# ArduPilot AP_Mount 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Mount.cpp" "AP_Mount_Alexmos.cpp" "AP_Mount_Backend.cpp" "AP_Mount_Backend_Serial.cpp" "AP_Mount_CADDX.cpp" "AP_Mount_Gremsy.cpp" "AP_Mount_Params.cpp" "AP_Mount_Scripting.cpp" "AP_Mount_Servo.cpp" "AP_Mount_Siyi.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Mount.cpp"
    "AP_Mount_Alexmos.cpp"
    "AP_Mount_Backend.cpp"
    "AP_Mount_Backend_Serial.cpp"
    "AP_Mount_CADDX.cpp"
    "AP_Mount_Gremsy.cpp"
    "AP_Mount_Params.cpp"
    "AP_Mount_Scripting.cpp"
    "AP_Mount_Servo.cpp"
    "AP_Mount_Siyi.cpp"
    "AP_Mount_SoloGimbal.cpp"
    "AP_Mount_SToRM32.cpp"
    "AP_Mount_SToRM32_serial.cpp"
    "AP_Mount_Topotek.cpp"
    "AP_Mount_Viewpro.cpp"
    "AP_Mount_Xacti.cpp"
    "AP_Mount_XFRobot.cpp"
    "SoloGimbal.cpp"
    "SoloGimbalEKF.cpp"
    "SoloGimbal_Parameters.cpp"
    "AP_Mount.cpp"
    "AP_Mount_Alexmos.cpp"
    "AP_Mount_Backend.cpp"
    "AP_Mount_Backend_Serial.cpp"
    "AP_Mount_CADDX.cpp"
    "AP_Mount_Gremsy.cpp"
    "AP_Mount_Params.cpp"
    "AP_Mount_Scripting.cpp"
    "AP_Mount_Servo.cpp"
    "AP_Mount_Siyi.cpp"
    "AP_Mount_SoloGimbal.cpp"
    "AP_Mount_SToRM32.cpp"
    "AP_Mount_SToRM32_serial.cpp"
    "AP_Mount_Topotek.cpp"
    "AP_Mount_Viewpro.cpp"
    "AP_Mount_Xacti.cpp"
    "AP_Mount_XFRobot.cpp"
    "SoloGimbal.cpp"
    "SoloGimbalEKF.cpp"
    "SoloGimbal_Parameters.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
