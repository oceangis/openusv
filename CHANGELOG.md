# Changelog

## [v2.0.0] - 2025-12-30

### 🎉 重大更新 - BNO08x IMU 稳定运行 & 翼帆控制框架

#### BNO08x ExternalAHRS 修复
- **I2C 设备探测增强**: 添加详细调试日志，便于定位通信问题
- **probe_device()**: 记录 bus/addr、get_device_ptr 状态、transfer 结果
- **hal_read()**: 添加 header 原始数据日志，便于协议分析

#### MAVLink 消息修复
- **MSG_WHEEL_DISTANCE (60) 修复**: 解决 "Sending unknown message (60)" 警告
  - 问题: 禁用 AP_WHEELENCODER_ENABLED=0 后，消息调度未同步禁用
  - 修复: GCS_MAVLink_Parameters.cpp 添加 && AP_WHEELENCODER_ENABLED 条件

#### 翼帆控制统一框架 (Sailboat Wingsail Framework)
- **三种翼帆类型支持**:
  - WINGSAIL_ROTATION: 舵+翼帆旋转 (控制整个翼帆角度)
  - WINGSAIL_FLAP: 舵+襟翼 (OpenTransat 方式)
  - WINGSAIL_FREE: 只控舵 (Sailbuoy 自平衡方式)
- **统一控制接口**:
  - get_wingsail_type() - 获取翼帆类型
  - get_normalized_control() - 归一化控制值 (-1.0 ~ +1.0)
  - get_steering_gain() - 模式相关舵增益 (ROTATION:0.8, FLAP:1.0, FREE:1.3)
  - get_steering_correction() - 统一舵修正值

#### INA2xx 电池监控优化
- **I2C 总线支持**: 支持指定 I2C 总线号
- **参数优化**: 电池监控参数调整

#### 修改的文件
- libraries/AP_ExternalAHRS/AP_ExternalAHRS_BNO08x.cpp - 调试日志
- libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp - MSG_WHEEL_DISTANCE 修复
- Rover/sailboat.h, Rover/sailboat.cpp - 翼帆框架
- Rover/mode.cpp, Rover/mode_manual.cpp - 舵控制集成
- libraries/AP_BattMonitor/AP_BattMonitor_INA2xx.* - 电池监控
- libraries/RC_Channel/RC_Channel.h - RC 通道优化

---

# Changelog

## [v1.7.0] - 2025-12-29

### MAVLink 带宽优化 (LoRa 遥测适配)

针对 LoRa 低带宽链路优化 MAVLink 数据传输，总带宽从 ~12 kbps 降至 ~4.2 kbps。

#### 禁用的 MAVLink 消息

| 消息 | 所属流 | 说明 |
|------|--------|------|
| MSG_AHRS | EXTRA3 | AHRS 调试信息 |
| MSG_AHRS2 | EXTRA1 | 备用 AHRS |
| MSG_EKF_STATUS_REPORT | EXTRA3 | EKF 状态 |
| MSG_VIBRATION | EXTRA3 | 振动数据 |
| MSG_LOCAL_POSITION | POSITION | 与 GPS 重复 |
| MSG_SYSTEM_TIME | EXTRA3 | 非关键 |
| MSG_DISTANCE_SENSOR | EXTRA3 | 用 WATER_DEPTH 代替 |
| MSG_MEMINFO | EXT_STAT | 内存调试 |
| RAW_SENS 流 | - | 原始 IMU 数据 |
| RAW_CTRL 流 | - | 原始控制输出 |

#### 保留的核心消息

- HEARTBEAT - 连接状态
- SYS_STATUS - 系统健康
- GPS_RAW - GPS 状态
- GLOBAL_POSITION_INT - 位置
- ATTITUDE - 姿态
- VFR_HUD - 速度/航向
- BATTERY_STATUS - 电池
- WATER_DEPTH - 测深仪
- SERVO_OUTPUT_RAW - 电机输出 (调试)
- RC_CHANNELS - RC 输入 (调试)

#### 新增功能

- AP_FENCE 启用 - 地理围栏安全功能

#### 修改的文件

- Rover/config.h - MAVLink 消息禁用宏
- libraries/AP_HAL/board/esp32.h - AP_FENCE_ENABLED=1
- libraries/GCS_MAVLink/GCS_config.h - 消息控制宏定义
- libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp - 条件编译
- hwdef/esp32s3rover/defaults.parm - 默认流速率
- params/usv_minimal.param - 参数参考文件

#### 带宽对比

| 配置 | 带宽 |
|------|------|
| 原始 Rover 默认 | ~12 kbps |
| v1.7.0 优化后 | ~4.2 kbps |
| 节省 | ~65% |

---

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
