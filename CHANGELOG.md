# Changelog

## [v1.2] - 2025-12-15

### 新增 (Added)
- **BNO08x ExternalAHRS 驱动**: 完整实现 BNO08x IMU 传感器支持
  - `AP_ExternalAHRS_BNO08x.cpp/h` - 基于 Adafruit/SparkFun 参考实现
  - SHTP 协议通信（I2C）
  - 支持旋转向量、陀螺仪、加速度计数据
  - Q-point 定点数转浮点转换
  - 设备类型 `EAHRS_TYPE = 11`
- **ExternalAHRS 框架启用**: 在 ESP32 平台启用外部 AHRS 支持
  - `AP_EXTERNAL_AHRS_ENABLED = 1`
  - `AP_EXTERNAL_AHRS_BNO08X_ENABLED = 1`

### 改进 (Improved)
- **AK09916 磁力计**: 优化初始化和数据读取流程
- **Storage 驱动**: 改进 NVS 存储实现
- **Scheduler**: 调整调度器配置
- **IMU 传感器**: 增强 Invensensev2 驱动调试信息
- **分区表**: 优化 Flash 分区布局

### 技术细节
- **BNO08x 使用方式**:
  - 设置 `EAHRS_TYPE = 11` 启用
  - I2C 地址: 0x4A (或 0x4B)
  - `AHRS_EKF_TYPE = 11`: 直接使用 BNO08x 融合姿态
  - `AHRS_EKF_TYPE = 3`: 原始数据送入 EKF3 滤波

---

## [v1.1] - 2025-11-12

### 修复 (Fixed)
- **修复AP_AHRS API兼容性**: 将已废弃的`get_yaw()`替换为`get_yaw_rad()`
- **修复未定义变量**: 修正`channel_yaw`为`channel_roll`
- **修复EKF函数调用**: 将`position_ok()`更正为`ekf_position_ok()`
- **修复友元类声明**: 添加ModePosHold为Rover的友元类

### 简化 (Simplified)
- **GPIO简化**: 移除过度设计的GPIO中断系统（200+行代码）

### 新增 (Added)
- **PosHold模式**: 为X型推进器船型添加位置保持模式
- **DroneCAN支持**: 保留完整的DroneCAN功能
- **板载配置**: esp32s3rover硬件定义

### 优化 (Improved)
- **Manual模式**: 为X型船添加航向辅助功能

---

## [v1.0] - 2024-12-XX

### 初始版本
- ArduPilot Rover ESP32-S3 IDF移植
- 基础功能实现
- ESP32精简版发布
