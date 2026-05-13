# ===========================================================================
# ESP32 Board Configuration
# 修改此文件选择板型，无需命令行参数
# ===========================================================================

# 可用板型 (对应 libraries/AP_HAL_ESP32/hwdef/ 下的目录):
#   - esp32s3rover_icm20948  : ICM-20948 + AK09916 + EKF3 (当前主力)
#   - esp32s3rover_bno08x    : BNO08x ExternalAHRS (备用)
#   - esp32s3rover           : 旧目录 (不推荐)
#
# 修改下面这行来切换板型:
set(ESP32_BOARD "esp32s3rover_icm20948")
