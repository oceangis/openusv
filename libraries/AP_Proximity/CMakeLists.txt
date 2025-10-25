# ArduPilot AP_Proximity 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Proximity.cpp" "AP_Proximity_AirSimSITL.cpp" "AP_Proximity_Backend.cpp" "AP_Proximity_Backend_Serial.cpp" "AP_Proximity_Boundary_3D.cpp" "AP_Proximity_Cygbot_D1.cpp" "AP_Proximity_DroneCAN.cpp" "AP_Proximity_LD06.cpp" "AP_Proximity_LightWareSF40C.cpp" "AP_Proximity_LightWareSF45B.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "AP_Proximity"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Proximity.cpp"
    "AP_Proximity_AirSimSITL.cpp"
    "AP_Proximity_Backend.cpp"
    "AP_Proximity_Backend_Serial.cpp"
    "AP_Proximity_Boundary_3D.cpp"
    "AP_Proximity_Cygbot_D1.cpp"
    "AP_Proximity_DroneCAN.cpp"
    "AP_Proximity_LD06.cpp"
    "AP_Proximity_LightWareSF40C.cpp"
    "AP_Proximity_LightWareSF45B.cpp"
    "AP_Proximity_MAV.cpp"
    "AP_Proximity_MR72_CAN.cpp"
    "AP_Proximity_Params.cpp"
    "AP_Proximity_RangeFinder.cpp"
    "AP_Proximity_RPLidarA2.cpp"
    "AP_Proximity_Scripting.cpp"
    "AP_Proximity_SITL.cpp"
    "AP_Proximity_TeraRangerTower.cpp"
    "AP_Proximity_TeraRangerTowerEvo.cpp"
    "AP_Proximity_Utils.cpp"
    "AP_Proximity.cpp"
    "AP_Proximity_AirSimSITL.cpp"
    "AP_Proximity_Backend.cpp"
    "AP_Proximity_Backend_Serial.cpp"
    "AP_Proximity_Boundary_3D.cpp"
    "AP_Proximity_Cygbot_D1.cpp"
    "AP_Proximity_DroneCAN.cpp"
    "AP_Proximity_LD06.cpp"
    "AP_Proximity_LightWareSF40C.cpp"
    "AP_Proximity_LightWareSF45B.cpp"
    "AP_Proximity_MAV.cpp"
    "AP_Proximity_MR72_CAN.cpp"
    "AP_Proximity_Params.cpp"
    "AP_Proximity_RangeFinder.cpp"
    "AP_Proximity_RPLidarA2.cpp"
    "AP_Proximity_Scripting.cpp"
    "AP_Proximity_SITL.cpp"
    "AP_Proximity_TeraRangerTower.cpp"
    "AP_Proximity_TeraRangerTowerEvo.cpp"
    "AP_Proximity_Utils.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "AP_Proximity"
)
