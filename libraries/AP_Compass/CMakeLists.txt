# ArduPilot AP_Compass 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Compass.cpp" "AP_Compass_AK09916.cpp" "AP_Compass_AK8963.cpp" "AP_Compass_Backend.cpp" "AP_Compass_BMM150.cpp" "AP_Compass_BMM350.cpp" "AP_Compass_Calibration.cpp" "AP_Compass_DroneCAN.cpp" "AP_Compass_ExternalAHRS.cpp" "AP_Compass_HMC5843.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Compass.cpp"
    "AP_Compass_AK09916.cpp"
    "AP_Compass_AK8963.cpp"
    "AP_Compass_Backend.cpp"
    "AP_Compass_BMM150.cpp"
    "AP_Compass_BMM350.cpp"
    "AP_Compass_Calibration.cpp"
    "AP_Compass_DroneCAN.cpp"
    "AP_Compass_ExternalAHRS.cpp"
    "AP_Compass_HMC5843.cpp"
    "AP_Compass_IIS2MDC.cpp"
    "AP_Compass_IST8308.cpp"
    "AP_Compass_IST8310.cpp"
    "AP_Compass_LIS3MDL.cpp"
    "AP_Compass_LSM303D.cpp"
    "AP_Compass_LSM9DS1.cpp"
    "AP_Compass_MAG3110.cpp"
    "AP_Compass_MMC3416.cpp"
    "AP_Compass_MMC5xx3.cpp"
    "AP_Compass_MSP.cpp"
    "AP_Compass_QMC5883L.cpp"
    "AP_Compass_QMC5883P.cpp"
    "AP_Compass_RM3100.cpp"
    "AP_Compass_SITL.cpp"
    "CompassCalibrator.cpp"
    "Compass_learn.cpp"
    "Compass_PerMotor.cpp"
    "AP_Compass.cpp"
    "AP_Compass_AK09916.cpp"
    "AP_Compass_AK8963.cpp"
    "AP_Compass_Backend.cpp"
    "AP_Compass_BMM150.cpp"
    "AP_Compass_BMM350.cpp"
    "AP_Compass_Calibration.cpp"
    "AP_Compass_DroneCAN.cpp"
    "AP_Compass_ExternalAHRS.cpp"
    "AP_Compass_HMC5843.cpp"
    "AP_Compass_IIS2MDC.cpp"
    "AP_Compass_IST8308.cpp"
    "AP_Compass_IST8310.cpp"
    "AP_Compass_LIS3MDL.cpp"
    "AP_Compass_LSM303D.cpp"
    "AP_Compass_LSM9DS1.cpp"
    "AP_Compass_MAG3110.cpp"
    "AP_Compass_MMC3416.cpp"
    "AP_Compass_MMC5xx3.cpp"
    "AP_Compass_MSP.cpp"
    "AP_Compass_QMC5883L.cpp"
    "AP_Compass_QMC5883P.cpp"
    "AP_Compass_RM3100.cpp"
    "AP_Compass_SITL.cpp"
    "CompassCalibrator.cpp"
    "Compass_learn.cpp"
    "Compass_PerMotor.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
