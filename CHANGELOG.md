# Changelog

## [v1.6.0] - 2025-12-29

### USV 功能精简 (USV Feature Optimization)

针对无人船 (USV) 应用场景，禁用不需要的陆地车辆和飞行器功能，减少代码体积和内存占用。

#### 已禁用的功能模块

| 功能 | 宏定义 | 参数组 | 说明 |
|------|--------|--------|------|
| 轮编码器 | AP_WHEELENCODER_ENABLED=0 | WENC | USV 无轮子 |
| 轮速率控制 | AP_WHEELRATECONTROL_ENABLED=0 | WRC | USV 无轮子 |
| 翻车检测 | AP_ROVER_CRASH_CHECK_ENABLED=0 | CRASH_ANGLE | USV 不会翻车 |
| 平衡车 | AP_ROVER_BALANCEBOT_ENABLED=0 | - | 两轮平衡车功能 |
| MSP 协议 | HAL_MSP_ENABLED=0 | - | Betaflight OSD 协议 |
| 室内信标 | AP_BEACON_ENABLED=0 | BCN | USV 户外使用 GPS |
| 喷洒器 | HAL_SPRAYER_ENABLED=0 | SPRAY | 农业喷洒功能 |
| 精确着陆 | AC_PRECLAND_ENABLED=0 | PLND | 飞行器功能 |
| 对接模式 | MODE_DOCK_ENABLED=0 | - | 依赖精确着陆 |
| 光流传感器 | AP_OPTICALFLOW_ENABLED=0 | FLOW | 飞行器悬停用 |
| 空速计 | AP_AIRSPEED_ENABLED=0 | - | 飞行器用 |

#### 修改的文件
- Rover/config.h - 添加所有禁用宏定义
- Rover/Parameters.cpp - 条件编译参数组
- Rover/Parameters.h - 条件编译成员变量
- Rover/Rover.cpp - 条件编译调度任务
- Rover/Rover.h - 条件编译函数声明
- Rover/sensors.cpp - 条件编译轮编码器更新
- Rover/crash_check.cpp - 条件编译整个文件
- Rover/system.cpp - 条件编译初始化代码
- Rover/sailboat.cpp - 条件编译默认值设置
- Rover/GCS_MAVLink_Rover.cpp - 条件编译 MAVLink 消息
- Rover/Log.cpp - 条件编译日志记录
- libraries/APM_Control/AR_AttitudeControl.cpp/h - 条件编译 Balance Bot PID

#### USV 保留的核心功能
- GPS 导航定位
- 罗盘航向
- IMU 姿态 (BNO08x ExternalAHRS)
- 电池监控
- MAVLink 通信 (UART/WiFi/LoRa)
- MODE_MANUAL - 手动控制
- MODE_AUTO - 自主航行
- MODE_RTL - 返航
- MODE_LOITER - 悬停定点
- MODE_GUIDED - 地面站引导
- Sailboat - 帆船功能

---

## [v1.5.0] - 2025-12-29

### 优化 (Improved)
- **禁用 Balance Bot 功能**: USV/船型不需要两轮平衡车功能
  - AP_ROVER_BALANCEBOT_ENABLED = 0
  - 减少代码体积和内存占用

- **串口映射优化**: 更直观的串口编号映射
  - SERIAL0 = UART0 (MAVLink)
  - SERIAL1 = UART1 (GPS) - 修复 GPS 无法识别问题
  - SERIAL3 = LoRa (虚拟串口，底层 SPI)

### 修复 (Fixed)
- **Balance Bot 条件编译**: 修复禁用 balance bot 后的编译错误
  - balance_bot.cpp - 添加 stub 函数
  - AR_AttitudeControl.cpp/h - 条件编译 pitch to throttle PID
  - Log.cpp - 条件编译 get_desired_pitch() 调用
  - GCS_MAVLink_Rover.cpp - 条件编译 balance bot 相关代码

- **GPS 串口配置**: 修复 GPS 数据无法读取问题
  - 原因: SERIAL1 映射到 Empty 驱动，GPS 配置在 SERIAL3
  - 修复: HAL_ESP32_Class.cpp 中 serial1Driver 改为 UARTDriver(1)

### 技术细节
- **串口映射** (HAL_ESP32_Class.cpp):
  - SERIAL0 = UARTDriver(0) // UART0, MAVLink
  - SERIAL1 = UARTDriver(1) // UART1, GPS @ 38400
  - SERIAL2 = Empty         // 无物理 UART
  - SERIAL3 = LoRaUARTDriver // 虚拟串口 (SPI->LoRa)

- **GPS 硬件**:
  - u-blox MAX-M10S @ UART1 (RX=GPIO18, TX=GPIO17)
  - 波特率: 38400

---

## [v1.4.0] - 2025-12-28

### 新增 (Added)
- **u-blox MAX-M10S GPS 支持**: 成功读取 GPS 数据
- **BNO08x IMU 数据读取**: 成功读取姿态数据

### 修复 (Fixed)
- **AHRS ExternalAHRS 集成**: 修复 BNO08x 姿态数据未被导航使用的问题

---

## [v1.3] - 2025-12-17

### 新增 (Added)
- **WiFi 配置系统**: 完整的 WiFi 参数集成到 ArduPilot
- **LoRa MAVLink 数传**: 基于 SX1262 的远程遥测系统

---

## [v1.2] - 2025-12-15

### 新增 (Added)
- **BNO08x ExternalAHRS 驱动**: 完整实现 BNO08x IMU 传感器支持
- **ExternalAHRS 框架启用**: 在 ESP32 平台启用外部 AHRS 支持

---

## [v1.1] - 2025-11-12

### 修复 (Fixed)
- 修复AP_AHRS API兼容性
- 修复EKF函数调用

### 新增 (Added)
- PosHold模式
- DroneCAN支持

---

## [v1.0] - 2024-12-XX

### 初始版本
- ArduPilot Rover ESP32-S3 IDF移植
- 基础功能实现
