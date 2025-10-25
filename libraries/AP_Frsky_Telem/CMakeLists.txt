# ArduPilot AP_Frsky_Telem 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Frsky_Backend.cpp" "AP_Frsky_D.cpp" "AP_Frsky_MAVliteMsgHandler.cpp" "AP_Frsky_MAVlite_MAVliteToSPort.cpp" "AP_Frsky_MAVlite_Message.cpp" "AP_Frsky_MAVlite_SPortToMAVlite.cpp" "AP_Frsky_Parameters.cpp" "AP_Frsky_SPort.cpp" "AP_Frsky_SPort_Passthrough.cpp" "AP_Frsky_Telem.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Frsky_Backend.cpp"
    "AP_Frsky_D.cpp"
    "AP_Frsky_MAVliteMsgHandler.cpp"
    "AP_Frsky_MAVlite_MAVliteToSPort.cpp"
    "AP_Frsky_MAVlite_Message.cpp"
    "AP_Frsky_MAVlite_SPortToMAVlite.cpp"
    "AP_Frsky_Parameters.cpp"
    "AP_Frsky_SPort.cpp"
    "AP_Frsky_SPort_Passthrough.cpp"
    "AP_Frsky_Telem.cpp"
    "AP_Frsky_Backend.cpp"
    "AP_Frsky_D.cpp"
    "AP_Frsky_MAVliteMsgHandler.cpp"
    "AP_Frsky_MAVlite_MAVliteToSPort.cpp"
    "AP_Frsky_MAVlite_Message.cpp"
    "AP_Frsky_MAVlite_SPortToMAVlite.cpp"
    "AP_Frsky_Parameters.cpp"
    "AP_Frsky_SPort.cpp"
    "AP_Frsky_SPort_Passthrough.cpp"
    "AP_Frsky_Telem.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
