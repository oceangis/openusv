# ArduPilot AP_PiccoloCAN 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_PiccoloCAN.cpp" "AP_PiccoloCAN_ECU.cpp" "AP_PiccoloCAN_ESC.cpp" "AP_PiccoloCAN_Servo.cpp" "AP_PiccoloCAN.cpp" "AP_PiccoloCAN_ECU.cpp" "AP_PiccoloCAN_ESC.cpp" "AP_PiccoloCAN_Servo.cpp" "piccolo_protocol/ECUDefines.c" "piccolo_protocol/ECUPackets.c"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "piccolo_protocol"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_PiccoloCAN.cpp"
    "AP_PiccoloCAN_ECU.cpp"
    "AP_PiccoloCAN_ESC.cpp"
    "AP_PiccoloCAN_Servo.cpp"
    "AP_PiccoloCAN.cpp"
    "AP_PiccoloCAN_ECU.cpp"
    "AP_PiccoloCAN_ESC.cpp"
    "AP_PiccoloCAN_Servo.cpp"
    "piccolo_protocol/ECUDefines.c"
    "piccolo_protocol/ECUPackets.c"
    "piccolo_protocol/ECUProtocol.c"
    "piccolo_protocol/ECUSettings.c"
    "piccolo_protocol/ESCCommands.c"
    "piccolo_protocol/ESCDefines.c"
    "piccolo_protocol/ESCPackets.c"
    "piccolo_protocol/ESCVelocityProtocol.c"
    "piccolo_protocol/fielddecode.c"
    "piccolo_protocol/fieldencode.c"
    "piccolo_protocol/scaleddecode.c"
    "piccolo_protocol/scaledencode.c"
    "piccolo_protocol/ServoCommands.c"
    "piccolo_protocol/ServoDefines.c"
    "piccolo_protocol/ServoPackets.c"
    "piccolo_protocol/ServoProtocol.c"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "piccolo_protocol"
)
