# ArduPilot AC_AutoTune 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AC_AutoTune.cpp" "AC_AutoTune_FreqResp.cpp" "AC_AutoTune_Heli.cpp" "AC_AutoTune_Multi.cpp" "AC_AutoTune.cpp" "AC_AutoTune_FreqResp.cpp" "AC_AutoTune_Heli.cpp" "AC_AutoTune_Multi.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AC_AutoTune.cpp"
    "AC_AutoTune_FreqResp.cpp"
    "AC_AutoTune_Heli.cpp"
    "AC_AutoTune_Multi.cpp"
    "AC_AutoTune.cpp"
    "AC_AutoTune_FreqResp.cpp"
    "AC_AutoTune_Heli.cpp"
    "AC_AutoTune_Multi.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
