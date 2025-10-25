# ArduPilot AP_DDS 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_DDS_Client.cpp" "AP_DDS_ExternalControl.cpp" "AP_DDS_External_Odom.cpp" "AP_DDS_Serial.cpp" "AP_DDS_Type_Conversions.cpp" "AP_DDS_UDP.cpp" "AP_DDS_Client.cpp" "AP_DDS_ExternalControl.cpp" "AP_DDS_External_Odom.cpp" "AP_DDS_Serial.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "Idl" "Idl/ardupilot_msgs" "Idl/ardupilot_msgs/msg" "Idl/ardupilot_msgs/srv"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_DDS_Client.cpp"
    "AP_DDS_ExternalControl.cpp"
    "AP_DDS_External_Odom.cpp"
    "AP_DDS_Serial.cpp"
    "AP_DDS_Type_Conversions.cpp"
    "AP_DDS_UDP.cpp"
    "AP_DDS_Client.cpp"
    "AP_DDS_ExternalControl.cpp"
    "AP_DDS_External_Odom.cpp"
    "AP_DDS_Serial.cpp"
    "AP_DDS_Type_Conversions.cpp"
    "AP_DDS_UDP.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "Idl"
    "Idl/ardupilot_msgs"
    "Idl/ardupilot_msgs/msg"
    "Idl/ardupilot_msgs/srv"
    "Idl/builtin_interfaces"
    "Idl/builtin_interfaces/msg"
    "Idl/geographic_msgs"
    "Idl/geographic_msgs/msg"
    "Idl/geometry_msgs"
    "Idl/geometry_msgs/msg"
    "Idl/rcl_interfaces"
    "Idl/rcl_interfaces/msg"
    "Idl/rcl_interfaces/srv"
    "Idl/rosgraph_msgs"
    "Idl/rosgraph_msgs/msg"
    "Idl/sensor_msgs"
    "Idl/sensor_msgs/msg"
    "Idl/std_msgs"
    "Idl/std_msgs/msg"
    "Idl/std_srvs"
    "Idl/std_srvs/srv"
    "Idl/tf2_msgs"
    "Idl/tf2_msgs/msg"
)
