# ArduPilot AP_Motors 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_Motors6DOF.cpp" "AP_MotorsCoax.cpp" "AP_MotorsHeli.cpp" "AP_MotorsHeli_Dual.cpp" "AP_MotorsHeli_Quad.cpp" "AP_MotorsHeli_RSC.cpp" "AP_MotorsHeli_Single.cpp" "AP_MotorsHeli_Swash.cpp" "AP_MotorsMatrix.cpp" "AP_MotorsMatrix_6DoF_Scripting.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "."  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_Motors6DOF.cpp"
    "AP_MotorsCoax.cpp"
    "AP_MotorsHeli.cpp"
    "AP_MotorsHeli_Dual.cpp"
    "AP_MotorsHeli_Quad.cpp"
    "AP_MotorsHeli_RSC.cpp"
    "AP_MotorsHeli_Single.cpp"
    "AP_MotorsHeli_Swash.cpp"
    "AP_MotorsMatrix.cpp"
    "AP_MotorsMatrix_6DoF_Scripting.cpp"
    "AP_MotorsMatrix_Scripting_Dynamic.cpp"
    "AP_MotorsMulticopter.cpp"
    "AP_MotorsSingle.cpp"
    "AP_MotorsTailsitter.cpp"
    "AP_MotorsTri.cpp"
    "AP_Motors_Class.cpp"
    "AP_Motors_Thrust_Linearization.cpp"
    "AP_Motors6DOF.cpp"
    "AP_MotorsCoax.cpp"
    "AP_MotorsHeli.cpp"
    "AP_MotorsHeli_Dual.cpp"
    "AP_MotorsHeli_Quad.cpp"
    "AP_MotorsHeli_RSC.cpp"
    "AP_MotorsHeli_Single.cpp"
    "AP_MotorsHeli_Swash.cpp"
    "AP_MotorsMatrix.cpp"
    "AP_MotorsMatrix_6DoF_Scripting.cpp"
    "AP_MotorsMatrix_Scripting_Dynamic.cpp"
    "AP_MotorsMulticopter.cpp"
    "AP_MotorsSingle.cpp"
    "AP_MotorsTailsitter.cpp"
    "AP_MotorsTri.cpp"
    "AP_Motors_Class.cpp"
    "AP_Motors_Thrust_Linearization.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
