# ArduPilot AR_WPNav 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AR_PivotTurn.cpp" "AR_WPNav.cpp" "AR_WPNav_OA.cpp" "AR_PivotTurn.cpp" "AR_WPNav.cpp" "AR_WPNav_OA.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AR_PivotTurn.cpp"
    "AR_WPNav.cpp"
    "AR_WPNav_OA.cpp"
    "AR_PivotTurn.cpp"
    "AR_WPNav.cpp"
    "AR_WPNav_OA.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
