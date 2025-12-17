# Changelog

## [v1.3] - 2025-12-17

### 新增 (Added)
- **WiFi 配置系统**: 完整的 WiFi 参数集成到 ArduPilot
  - `components/wifi_config/` - ESP-IDF WiFi 配置组件
  - `libraries/AP_WiFi_ESP32/` - ArduPilot 参数集成类
  - 支持 AP/STA/APSTA 模式切换
  - MAVLink UDP 转发功能
  - Web 配置界面
  - **参数前缀**: `WIFI_`
    - `WIFI_ENABLE` - 启用/禁用 WiFi
    - `WIFI_MODE` - 工作模式 (0:OFF, 1:AP, 2:STA, 3:APSTA)
    - `WIFI_AP_CHAN` - AP 信道
    - `WIFI_MAV_EN` - MAVLink UDP 启用
    - `WIFI_MAV_PORT` - MAVLink UDP 端口

- **LoRa MAVLink 数传**: 基于 SX1262 的远程遥测系统
  - `components/lora_mavlink/` - LoRa 通信组件 (SX1262 驱动)
  - `libraries/AP_LoRa_ESP32/` - ArduPilot 参数集成类
  - `libraries/AP_HAL_ESP32/LoRaUARTDriver.*` - 虚拟 UART 驱动
  - 作为 SERIAL4 集成到 ArduPilot 串口系统
  - 支持 3-5km 远程 MAVLink 通信
  - **参数前缀**: `LORA_`
    - `LORA_ENABLE` - 启用/禁用 LoRa
    - `LORA_FREQ` - 频率 (kHz, 默认 433000)
    - `LORA_POWER` - 发射功率 (dBm, 2-22)
    - `LORA_SF` - 扩频因子 (6-12, 默认 8)
    - `LORA_BW` - 带宽 (0:125kHz, 1:250kHz, 2:500kHz)
    - `LORA_CR` - 编码率 (1:4/5 到 4:4/8)

### 改进 (Improved)
- **参数系统**: 统一使用 AP_Param (Flash 分区 0x45) 作为唯一配置源
- **main.c**: 优化启动顺序，先初始化 WiFi 和 LoRa，再启动 ArduPilot
- **CMakeLists**: 添加 wifi_config 和 lora_mavlink 组件依赖

### 技术细节
- **WiFi 使用方式**:
  - 默认 AP 模式，SSID: `USV_XXXXXX`
  - 连接后访问 `192.168.4.1` 进行 Web 配置
  - MissionPlanner 通过 UDP 14550 端口连接

- **LoRa 使用方式**:
  - 配置 `SERIAL4_PROTOCOL = 2` (MAVLink2)
  - 地面站需配对相同频率/SF/BW 的 LoRa 模块
  - 数据率约 800 bytes/s (SF8/125kHz)

### 硬件引脚 (esp32s3rover)
- **LoRa SX1262**:
  - SCK: GPIO35, MISO: GPIO36, MOSI: GPIO37
  - CS: GPIO39, RST: GPIO42, BUSY: GPIO40, DIO1: GPIO41

---

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
