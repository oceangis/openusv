# Changelog

## [v1.1] - 2025-11-12

### 修复 (Fixed)
- **修复AP_AHRS API兼容性**: 将已废弃的`get_yaw()`替换为`get_yaw_rad()`
  - AR_AttitudeControl.cpp:1165
  - mode_poshold.cpp:79, 130
- **修复未定义变量**: 修正`channel_yaw`为`channel_roll`（X型船使用roll通道控制yaw）
  - mode_manual.cpp:58
  - mode_poshold.cpp:165-166
- **修复EKF函数调用**: 将`position_ok()`更正为`ekf_position_ok()`
  - mode_poshold.cpp:24
- **修复友元类声明**: 添加ModePosHold为Rover的友元类，解决私有成员访问问题
  - Rover.h:109

### 简化 (Simplified)
- **GPIO简化**: 移除过度设计的GPIO中断系统（200+行代码）
  - 采用ArduPilot主项目的简洁实现（46行）
  - 移除了不必要的`attach_interrupt`和`detach_interrupt`功能
  - 删除了错误的GPIO_INTERRUPT_EXAMPLE.md文档
  - 遵循"不要过度设计，满足rover的功能即可"原则

### 新增 (Added)
- **PosHold模式**: 为X型推进器船型添加位置保持模式
  - 基于ArduSub的位置保持实现
  - 支持GPS位置保持
  - 支持ArduSub风格的航向保持（250ms平滑减速）
  - 体坐标系速度输入
- **DroneCAN支持**: 保留完整的DroneCAN功能
  - CANIface.cpp/h
  - Flash.cpp/h (OTA支持)
- **板载配置**: esp32s3rover硬件定义
  - ESP32-S3-N16R8 (16MB Flash, 8MB PSRAM)
  - ICM20948 IMU
  - DroneCAN总线
  - 4路PWM电机输出

### 优化 (Improved)
- **Manual模式**: 为X型船添加航向辅助功能
  - 模仿ArduSub的航向保持特性
  - 250ms平滑减速过渡
  - 绝对航向锁定

### 技术细节
- **基础版本**: ArduRover V4.7.0-dev
- **ESP-IDF**: 5.5.1
- **硬件**: ESP32-S3-N16R8
- **编译状态**: ✅ 成功通过编译验证

### 开发原则
- 遵循"ultrathink"原则：不过度设计
- 仅保留Rover必需功能
- 代码简洁、高效、稳定

---

## [v1.0] - 2024-12-XX

### 初始版本
- ArduPilot Rover ESP32-S3 IDF移植
- 基础功能实现
- ESP32精简版发布
