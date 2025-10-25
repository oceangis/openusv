# ArduPilot AP_Filesystem 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Filesystem.cpp" "AP_Filesystem_backend.cpp" "AP_Filesystem_ESP32.cpp" "AP_Filesystem_FATFS.cpp" "AP_Filesystem_FlashMemory_LittleFS.cpp" "AP_Filesystem_Mission.cpp" "AP_Filesystem_Param.cpp" "AP_Filesystem_posix.cpp" "AP_Filesystem_ROMFS.cpp" "AP_Filesystem_Sys.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Filesystem.cpp"
    "AP_Filesystem_backend.cpp"
    "AP_Filesystem_ESP32.cpp"
    "AP_Filesystem_FATFS.cpp"
    "AP_Filesystem_FlashMemory_LittleFS.cpp"
    "AP_Filesystem_Mission.cpp"
    "AP_Filesystem_Param.cpp"
    "AP_Filesystem_posix.cpp"
    "AP_Filesystem_ROMFS.cpp"
    "AP_Filesystem_Sys.cpp"
    "posix_compat.cpp"
    "AP_Filesystem.cpp"
    "AP_Filesystem_backend.cpp"
    "AP_Filesystem_ESP32.cpp"
    "AP_Filesystem_FATFS.cpp"
    "AP_Filesystem_FlashMemory_LittleFS.cpp"
    "AP_Filesystem_Mission.cpp"
    "AP_Filesystem_Param.cpp"
    "AP_Filesystem_posix.cpp"
    "AP_Filesystem_ROMFS.cpp"
    "AP_Filesystem_Sys.cpp"
    "posix_compat.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
