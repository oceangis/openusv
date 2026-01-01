# Changelog

## [v2.2.0] - 2026-01-02

### LoRa MAVLink 数传初步完成

#### LoRa SX1268 驱动集成
- **lora_mavlink 组件**: 完整的 TX/RX 实现
  - 环形缓冲区 (TX/RX ring buffer)
  - FreeRTOS 任务调度
  - DIO1 中断处理
- **sx126x 驱动配置** (EBYTE E22-400MBL 兼容):
  - Sync Word: 0x1444 (EBYTE 格式)
  - OCP: 0x38 (140mA)
  - TX Ramp: 40us
  - 433MHz, SF11, BW500kHz, 22dBm

#### SERIAL3 虚拟串口启用
- **架构**: SERIAL3 → LoRaUARTDriver → lora_mavlink → sx126x → SPI → SX1268
- **配置更新**:
  - HAL_UART_NUM_SERIAL_PORTS: 3 → 4
  - HAL_HAVE_SERIAL3_PARAMS: 0 → 1
  - DEFAULT_SERIAL3_PROTOCOL: SerialProtocol_MAVLink2
  - DEFAULT_SERIAL3_BAUD: 57600

#### hwdef.dat LoRa 引脚修复
- **问题**: SCK/MISO 引脚定义错误 (互换)
- **修复**:
  - LORA_PIN_SCK: GPIO_NUM_39 → GPIO_NUM_40
  - LORA_PIN_MISO: GPIO_NUM_40 → GPIO_NUM_39

#### Storage 调试输出修复
- **问题**: STORAGEDEBUG 宏启用导致大量 printf 输出
- **症状**: 参数加载时 UART 阻塞，触发看门狗超时重启 (rst:0xc)
- **修复**: 注释 #define STORAGEDEBUG 1
- **注意**: 此修复仅影响调试输出，参数存储功能不受影响

#### 修改的文件
- components/lora_mavlink/src/lora_mavlink.c - TX/RX 实现
- components/lora_mavlink/src/sx126x.c - EBYTE 兼容配置
- libraries/AP_HAL_ESP32/hwdef/esp32s3rover/hwdef.dat - 引脚修复 + SERIAL3
- libraries/AP_HAL_ESP32/hwdef/hwdef.h - SERIAL3 参数
- libraries/AP_HAL_ESP32/Storage.cpp - 禁用调试输出
- main/main.c - LoRa 初始化

#### 已知问题
- LoRa 收发功能需要实际测试验证
- Mission Planner 通过 LoRa 连接待测试

---

## [v2.1.0] - 2025-12-30

### BNO08x ExternalAHRS 重大修复

#### ENU→NED 坐标转换修复
- **问题**: 原四元数转换使用简单分量交换，数学上错误
- **修复**: 改用欧拉角中间转换，确保姿态数据正确
  - roll_ned = pitch_enu
  - pitch_ned = roll_enu  
  - yaw_ned = π/2 - yaw_enu (航向从东基准转北基准)

#### I2C 通信稳定性增强
- **问题**: 运行中 I2C 失败时无诊断信息，无法恢复
- **修复**:
  - 添加连续失败计数 consecutive_failures
  - 超过 50 次失败后自动重新初始化 (soft_reset + configure)
  - 每 10 秒输出统计日志: pkts/err/rst/reinit

#### 新增变量和常量
- consecutive_failures - I2C 连续失败计数
- reinit_count - 重新初始化次数
- MAX_CONSECUTIVE_FAILURES = 50
- STATS_INTERVAL_MS = 10000

#### 修改的文件
- libraries/AP_ExternalAHRS/AP_ExternalAHRS_BNO08x.h
- libraries/AP_ExternalAHRS/AP_ExternalAHRS_BNO08x.cpp

---

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
