# ArduPilot AP_OpticalFlow 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_OpticalFlow.cpp" "AP_OpticalFlow_Backend.cpp" "AP_OpticalFlow_Calibrator.cpp" "AP_OpticalFlow_CXOF.cpp" "AP_OpticalFlow_HereFlow.cpp" "AP_OpticalFlow_MAV.cpp" "AP_OpticalFlow_MSP.cpp" "AP_OpticalFlow_Onboard.cpp" "AP_OpticalFlow_Pixart.cpp" "AP_OpticalFlow_PX4Flow.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_OpticalFlow.cpp"
    "AP_OpticalFlow_Backend.cpp"
    "AP_OpticalFlow_Calibrator.cpp"
    "AP_OpticalFlow_CXOF.cpp"
    "AP_OpticalFlow_HereFlow.cpp"
    "AP_OpticalFlow_MAV.cpp"
    "AP_OpticalFlow_MSP.cpp"
    "AP_OpticalFlow_Onboard.cpp"
    "AP_OpticalFlow_Pixart.cpp"
    "AP_OpticalFlow_PX4Flow.cpp"
    "AP_OpticalFlow_SITL.cpp"
    "AP_OpticalFlow_UPFLOW.cpp"
    "AP_OpticalFlow.cpp"
    "AP_OpticalFlow_Backend.cpp"
    "AP_OpticalFlow_Calibrator.cpp"
    "AP_OpticalFlow_CXOF.cpp"
    "AP_OpticalFlow_HereFlow.cpp"
    "AP_OpticalFlow_MAV.cpp"
    "AP_OpticalFlow_MSP.cpp"
    "AP_OpticalFlow_Onboard.cpp"
    "AP_OpticalFlow_Pixart.cpp"
    "AP_OpticalFlow_PX4Flow.cpp"
    "AP_OpticalFlow_SITL.cpp"
    "AP_OpticalFlow_UPFLOW.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
