# ArduPilot AP_NavEKF 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_NavEKF_core_common.cpp" "AP_NavEKF_Source.cpp" "EKFGSF_Logging.cpp" "EKFGSF_yaw.cpp" "EKF_Buffer.cpp" "AP_NavEKF_core_common.cpp" "AP_NavEKF_Source.cpp" "EKFGSF_Logging.cpp" "EKFGSF_yaw.cpp" "EKF_Buffer.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "Models" "Models/AttErrVecMathExample" "Models/Common" "Models/GimbalEstimatorExample"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_NavEKF_core_common.cpp"
    "AP_NavEKF_Source.cpp"
    "EKFGSF_Logging.cpp"
    "EKFGSF_yaw.cpp"
    "EKF_Buffer.cpp"
    "AP_NavEKF_core_common.cpp"
    "AP_NavEKF_Source.cpp"
    "EKFGSF_Logging.cpp"
    "EKFGSF_yaw.cpp"
    "EKF_Buffer.cpp"
    "Models/GimbalEstimatorExample/calcF.cpp"
    "Models/GimbalEstimatorExample/calcH_MAG.cpp"
    "Models/GimbalEstimatorExample/calcP.cpp"
    "Models/GimbalEstimatorExample/calcQ.cpp"
    "Models/GimbalEstimatorExample/calcF.c"
    "Models/GimbalEstimatorExample/calcH_MAG.c"
    "Models/GimbalEstimatorExample/calcP.c"
    "Models/GimbalEstimatorExample/calcQ.c"
    "Models/GimbalEstimatorExample/calcSF.c"
    "Models/GimbalEstimatorExample/calcSP.c"
    "Models/GimbalEstimatorExample/calcTmn.c"
    "Models/GimbalEstimatorExample/calcTms.c"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "Models"
    "Models/AttErrVecMathExample"
    "Models/Common"
    "Models/GimbalEstimatorExample"
    "Models/QuaternionMathExample"
    "Models/testData"
)
