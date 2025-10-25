# ArduPilot AP_InertialSensor 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_InertialSensor.cpp" "AP_InertialSensor_ADIS1647x.cpp" "AP_InertialSensor_Backend.cpp" "AP_InertialSensor_BMI055.cpp" "AP_InertialSensor_BMI088.cpp" "AP_InertialSensor_BMI160.cpp" "AP_InertialSensor_BMI270.cpp" "AP_InertialSensor_ExternalAHRS.cpp" "AP_InertialSensor_Invensense.cpp" "AP_InertialSensor_Invensensev2.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_InertialSensor.cpp"
    "AP_InertialSensor_ADIS1647x.cpp"
    "AP_InertialSensor_Backend.cpp"
    "AP_InertialSensor_BMI055.cpp"
    "AP_InertialSensor_BMI088.cpp"
    "AP_InertialSensor_BMI160.cpp"
    "AP_InertialSensor_BMI270.cpp"
    "AP_InertialSensor_ExternalAHRS.cpp"
    "AP_InertialSensor_Invensense.cpp"
    "AP_InertialSensor_Invensensev2.cpp"
    "AP_InertialSensor_Invensensev3.cpp"
    "AP_InertialSensor_L3G4200D.cpp"
    "AP_InertialSensor_Logging.cpp"
    "AP_InertialSensor_LSM9DS0.cpp"
    "AP_InertialSensor_LSM9DS1.cpp"
    "AP_InertialSensor_NONE.cpp"
    "AP_InertialSensor_Params.cpp"
    "AP_InertialSensor_RST.cpp"
    "AP_InertialSensor_SCHA63T.cpp"
    "AP_InertialSensor_SITL.cpp"
    "AP_InertialSensor_tempcal.cpp"
    "AuxiliaryBus.cpp"
    "BatchSampler.cpp"
    "FastRateBuffer.cpp"
    "AP_InertialSensor.cpp"
    "AP_InertialSensor_ADIS1647x.cpp"
    "AP_InertialSensor_Backend.cpp"
    "AP_InertialSensor_BMI055.cpp"
    "AP_InertialSensor_BMI088.cpp"
    "AP_InertialSensor_BMI160.cpp"
    "AP_InertialSensor_BMI270.cpp"
    "AP_InertialSensor_ExternalAHRS.cpp"
    "AP_InertialSensor_Invensense.cpp"
    "AP_InertialSensor_Invensensev2.cpp"
    "AP_InertialSensor_Invensensev3.cpp"
    "AP_InertialSensor_L3G4200D.cpp"
    "AP_InertialSensor_Logging.cpp"
    "AP_InertialSensor_LSM9DS0.cpp"
    "AP_InertialSensor_LSM9DS1.cpp"
    "AP_InertialSensor_NONE.cpp"
    "AP_InertialSensor_Params.cpp"
    "AP_InertialSensor_RST.cpp"
    "AP_InertialSensor_SCHA63T.cpp"
    "AP_InertialSensor_SITL.cpp"
    "AP_InertialSensor_tempcal.cpp"
    "AuxiliaryBus.cpp"
    "BatchSampler.cpp"
    "FastRateBuffer.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
