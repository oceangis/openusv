# ArduPilot AP_ADSB 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_ADSB.cpp" "AP_ADSB_Backend.cpp" "AP_ADSB_Sagetech.cpp" "AP_ADSB_Sagetech_MXS.cpp" "AP_ADSB_uAvionix_MAVLink.cpp" "AP_ADSB_uAvionix_UCP.cpp" "AP_ADSB.cpp" "AP_ADSB_Backend.cpp" "AP_ADSB_Sagetech.cpp" "AP_ADSB_Sagetech_MXS.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "GDL90_protocol" "sagetech-sdk"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_ADSB.cpp"
    "AP_ADSB_Backend.cpp"
    "AP_ADSB_Sagetech.cpp"
    "AP_ADSB_Sagetech_MXS.cpp"
    "AP_ADSB_uAvionix_MAVLink.cpp"
    "AP_ADSB_uAvionix_UCP.cpp"
    "AP_ADSB.cpp"
    "AP_ADSB_Backend.cpp"
    "AP_ADSB_Sagetech.cpp"
    "AP_ADSB_Sagetech_MXS.cpp"
    "AP_ADSB_uAvionix_MAVLink.cpp"
    "AP_ADSB_uAvionix_UCP.cpp"
    "sagetech-sdk/appendChecksum.c"
    "sagetech-sdk/calcChecksum.c"
    "sagetech-sdk/charArray2Buf.c"
    "sagetech-sdk/float2Buf.c"
    "sagetech-sdk/icao2Buf.c"
    "sagetech-sdk/sgDecodeAck.c"
    "sagetech-sdk/sgDecodeFlightId.c"
    "sagetech-sdk/sgDecodeInstall.c"
    "sagetech-sdk/sgDecodeMSR.c"
    "sagetech-sdk/sgDecodeSVR.c"
    "sagetech-sdk/sgEncodeDataReq.c"
    "sagetech-sdk/sgEncodeFlightId.c"
    "sagetech-sdk/sgEncodeGPS.c"
    "sagetech-sdk/sgEncodeInstall.c"
    "sagetech-sdk/sgEncodeOperating.c"
    "sagetech-sdk/sgEncodeTargetReq.c"
    "sagetech-sdk/target.c"
    "sagetech-sdk/toAlt.c"
    "sagetech-sdk/toGS.c"
    "sagetech-sdk/toHeading.c"
    "sagetech-sdk/toHeading2.c"
    "sagetech-sdk/toIcao.c"
    "sagetech-sdk/toInt16.c"
    "sagetech-sdk/toInt32.c"
    "sagetech-sdk/toLatLon.c"
    "sagetech-sdk/toTOA.c"
    "sagetech-sdk/toUint16.c"
    "sagetech-sdk/toUint32.c"
    "sagetech-sdk/toVel.c"
    "sagetech-sdk/uint162Buf.c"
    "sagetech-sdk/uint322Buf.c"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "GDL90_protocol"
    "sagetech-sdk"
)
