# ArduPilot AC_Avoidance 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AC_Avoid.cpp" "AC_Avoidance_Logging.cpp" "AP_OABendyRuler.cpp" "AP_OADatabase.cpp" "AP_OADijkstra.cpp" "AP_OAPathPlanner.cpp" "AP_OAVisGraph.cpp" "AC_Avoid.cpp" "AC_Avoidance_Logging.cpp" "AP_OABendyRuler.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AC_Avoid.cpp"
    "AC_Avoidance_Logging.cpp"
    "AP_OABendyRuler.cpp"
    "AP_OADatabase.cpp"
    "AP_OADijkstra.cpp"
    "AP_OAPathPlanner.cpp"
    "AP_OAVisGraph.cpp"
    "AC_Avoid.cpp"
    "AC_Avoidance_Logging.cpp"
    "AP_OABendyRuler.cpp"
    "AP_OADatabase.cpp"
    "AP_OADijkstra.cpp"
    "AP_OAPathPlanner.cpp"
    "AP_OAVisGraph.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
