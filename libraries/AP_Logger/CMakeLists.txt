# ArduPilot AP_Logger 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Logger.cpp" "AP_Logger_Backend.cpp" "AP_Logger_Block.cpp" "AP_Logger_File.cpp" "AP_Logger_Flash_JEDEC.cpp" "AP_Logger_MAVLink.cpp" "AP_Logger_MAVLinkLogTransfer.cpp" "AP_Logger_W25NXX.cpp" "LogFile.cpp" "LoggerMessageWriter.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Logger.cpp"
    "AP_Logger_Backend.cpp"
    "AP_Logger_Block.cpp"
    "AP_Logger_File.cpp"
    "AP_Logger_Flash_JEDEC.cpp"
    "AP_Logger_MAVLink.cpp"
    "AP_Logger_MAVLinkLogTransfer.cpp"
    "AP_Logger_W25NXX.cpp"
    "LogFile.cpp"
    "LoggerMessageWriter.cpp"
    "AP_Logger.cpp"
    "AP_Logger_Backend.cpp"
    "AP_Logger_Block.cpp"
    "AP_Logger_File.cpp"
    "AP_Logger_Flash_JEDEC.cpp"
    "AP_Logger_MAVLink.cpp"
    "AP_Logger_MAVLinkLogTransfer.cpp"
    "AP_Logger_W25NXX.cpp"
    "LogFile.cpp"
    "LoggerMessageWriter.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
