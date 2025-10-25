# ArduPilot AP_Camera 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Camera.cpp" "AP_Camera_Backend.cpp" "AP_Camera_Logging.cpp" "AP_Camera_MAVLink.cpp" "AP_Camera_MAVLinkCamV2.cpp" "AP_Camera_Mount.cpp" "AP_Camera_Params.cpp" "AP_Camera_Relay.cpp" "AP_Camera_Scripting.cpp" "AP_Camera_Servo.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Camera.cpp"
    "AP_Camera_Backend.cpp"
    "AP_Camera_Logging.cpp"
    "AP_Camera_MAVLink.cpp"
    "AP_Camera_MAVLinkCamV2.cpp"
    "AP_Camera_Mount.cpp"
    "AP_Camera_Params.cpp"
    "AP_Camera_Relay.cpp"
    "AP_Camera_Scripting.cpp"
    "AP_Camera_Servo.cpp"
    "AP_Camera_SoloGimbal.cpp"
    "AP_RunCam.cpp"
    "AP_Camera.cpp"
    "AP_Camera_Backend.cpp"
    "AP_Camera_Logging.cpp"
    "AP_Camera_MAVLink.cpp"
    "AP_Camera_MAVLinkCamV2.cpp"
    "AP_Camera_Mount.cpp"
    "AP_Camera_Params.cpp"
    "AP_Camera_Relay.cpp"
    "AP_Camera_Scripting.cpp"
    "AP_Camera_Servo.cpp"
    "AP_Camera_SoloGimbal.cpp"
    "AP_RunCam.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
