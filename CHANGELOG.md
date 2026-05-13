# Changelog

## [v2.6.0] - 2026-05-13

### 固件标识 + 双推差速 USV 配置 + ACRO 航向保持

#### 固件 banner 重命名
- `Rover/version.h:9` — `THISFIRMWARE` 改为 `"ArduRover-ESP32S3 V1.0"`
- `libraries/AP_HAL_ESP32/hwdef/hwdef.h:93` — `HAL_ESP32_BOARD_NAME` 改为 `"ArduRover-ESP32S3 V1.0"`
- `libraries/AP_HAL_ESP32/hwdef/esp32s3rover_icm20948/hwdef.dat:6` — 同上
- **关键**：`FIRMWARE_VERSION 4,7,0,FIRMWARE_VERSION_TYPE_DEV` 数字**故意保留不变**，避免 ArduPilot "firmware change → erase EEPROM" 触发，所有用户参数（含校准）跨版本保留
- 启动通过 `MAV_CMD_DO_SEND_BANNER` (42428) 触发显示

#### ACRO 模式恢复（航向保持 / 转向速率控制）
- `libraries/AP_HAL_ESP32/hwdef/hwdef.h:65` `MODE_ACRO_ENABLED 0 → 1`
- `libraries/AP_HAL_ESP32/hwdef/esp32s3rover_common/base.dat:129` 同步
- 实测：摇杆居中时航向自锁，目标 turn rate=0 → IMU 闭环维持当前航向
- 默认参数：`ACRO_TURN_RATE=180°/s`, `ATC_STR_RAT_P/I=0.2`, `ATC_STR_RAT_FF=1.0`, `IMAX=1.0`
- 用途：USV 长途巡航的主力模式（只推油门，航向自动保持）

#### hwdef.h ↔ base.dat 一致性修复
- `AP_WINDVANE_ENABLED`: base.dat `0 → 1`（与 hwdef.h 一致）
- `RANGEFINDER_MAX_INSTANCES`: base.dat `0 → 1`（与 hwdef.h 一致）
- `LORA_SF`: 两处都 `11 → 7`（匹配 lora_mavlink 组件实际 SF7 配置）
- 避免下次 CMake 重生成 hwdef.h 时静默丢失修改

#### 差速双推 USV 配置（运行时参数，已写入 NVS）
- `SERVO1_FUNCTION = 73` (ThrottleLeft) → 输出到 GPIO 12 / H10 pin 5
- `SERVO3_FUNCTION = 74` (ThrottleRight) → 输出到 GPIO 45 / H10 pin 1
- `have_skid_steering()` 自动激活，差速混控公式 `S1=throttle+steer, S3=throttle-steer`
- 验证：50Hz 持续 RC override 测试 (`bench/verify/skid_steering.py`)：
  - 全右转 (1900,1500): DIFF=+758μs（原地右旋，一推正一推反）
  - 全左转 (1100,1500): DIFF=-750μs
  - 中位 (1500,1500): DIFF=0μs
  - 全部输出在 PWM [1100, 1900] 范围内

#### 6 面加速度校准（已固化）
- `INS_ACCOFFS_X/Y/Z = (-0.1355, +0.1508, +0.1462)` m/s²
- `INS_ACCSCAL_X/Y/Z ≈ 1.000` (scale 误差 < 0.3%)
- `AHRS_TRIM_X/Y = (-0.35°, +0.66°)`
- 静态 `|accel| = 999.3 mg` ≈ 1g
- 校准过程通过 `MAV_CMD_PREFLIGHT_CALIBRATION param5=1` + 6 次姿态确认完成

#### 配置参数固化
- `FS_THR_ENABLE = 0`（无 RC 接收机 → 关闭 RC failsafe）
- `LOG_BACKEND_TYPE = 0`（关闭 SD 日志失败 prearm 警告）
- `ARMING_CHECK = 178` = Baro|INS|Params|BoardVoltage（USV 测试组合）
- `WP_SPEED = 2.0`（对齐 CRUISE_SPEED，避免 base-throttle 外推）

#### AK09916 罗盘 I2C bypass 修复
- `libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.cpp:160-172, 802-810`
- 配置 ICM-20948 让 AK09916 透过 bypass 模式直接出现在主 I2C 总线（0x0C）
- `I2C_MST_EN=0` + `BIT_BYPASS_EN=1` 必须保持，FIFO reset 后需重新断言
- 与现行 `_probe_external_i2c_compasses()` 默认 `ROTATION_YAW_270` 不兼容，需在 hwdef.h 中显式 `HAL_SKIP_AUTO_INTERNAL_I2C_PROBE` + `HAL_MAG_PROBE_LIST AK09916:probe_ICM20948_I2C ROTATION_PITCH_180_YAW_90`

#### LoRa MAVLink 重写（components/lora_mavlink/）
- RX 主导状态机：默认 RX 监听，每 500ms 发一个遥测包（2Hz）
- 双 TX 队列：高优先级（HB/STATUSTEXT/COMMAND_ACK）即时发送，低优先级遥测攒包
- MAVLink 边界对齐：try_transmit 不在 LoRa 包间分割消息
- 高优先级消息列表加入 MISSION_REQUEST (msg_id=40) — ArduPilot 默认旧协议
- SF7/BW500: 10ms 包空中时间，RX 占空比 >98%
- 与 EBYTE E22-400MBL-SC 评估板互通 100% 已验证

#### Storage / FlashStorage 日志清理
- `libraries/AP_FlashStorage/AP_FlashStorage.cpp:24` — 禁用 `FLASHSTORAGE_DEBUG`（printf 在 UART0 上会与 HAL console 抢占造成死锁）
- `libraries/AP_HAL_ESP32/Storage.cpp` — `printf` → `ESP_LOGI`/`ESP_LOGE` 经 IDF 日志路由

#### 工具 / 文档
- 新建 `bench/` 目录归集出水测试脚本（verify/analyze/flash）
- 新建 `bench/README.md`、`params/README.md` 解释脚本和参数备份用途
- 新建 `PRE_WATER_CHECKLIST.md`（完整出水前流程）
- `params/baseline_v2.parm` 提交 529 参数完整快照
- `.gitignore` 大幅扩展（`/_*.py /_*.log /_*.txt /_*.json /_*.bat /_*.bin /_io_xlsx/ /baseline*.parm`）
- 删除 `Rover/balance_bot.cpp` 及其引用（USV 不需要平衡车代码）

---

## [v2.5.0] - 2026-01-15

### PCB V2.1 适配
- 修复重启死循环及硬件引脚配置

---

## [v2.4.0] - 2026-01-02

### USV 帆船仿真测试框架

#### Phase 1: C++ SITL 仿真器 (simulation/)
- **SIM_Sailboat_USV**: 完整的物理仿真引擎
  - 三种翼帆控制模式: ROTATION, FLAP, FREE
  - 气动升力/阻力曲线 (18点查表)
  - 船体水动力学
  - 波浪效应 (横摇/纵摇/升沉)
  - 潮流/海流模拟
- **环境参数**: 蒲福风级 (0-12), 道格拉斯海况 (0-9)
- **预设场景**: calm_sea, moderate_wind, storm, tide_current, upwind_test

#### Phase 2: Python 测试框架 (tests/sitl/)
- **sailboat_sim.py**: 纯 Python 物理模拟器
- **测试类别**: upwind_tacking, wingsail_modes, storm_stability
- **报告格式**: HTML/JSON/Markdown

#### Phase 3: GitHub Actions CI
- **sailboat-tests.yml**: Python 测试自动化
- **esp32-build.yml**: ESP32 固件编译检查

---

## [v2.3.0] - 2026-01-02

### 修复参数持久化关键BUG

- **问题**: AP_FlashStorage::init() 从 Sector 1 加载数据后，current_sector 仍为 0
- **后果**: 新参数写入 Sector 0，重启后加载 Sector 1 旧数据，导致参数丢失
- **修复**: 在 load_sector() 循环中记录 IN_USE 的 sector，加载完成后正确设置 current_sector
- 显式初始化构造函数成员变量，避免未定义初始值

---

## [v2.2.0] - 2026-01-02

### LoRa MAVLink 数传初步完成

- **lora_mavlink 组件**: TX/RX 实现 (环形缓冲区 + FreeRTOS 任务)
- **sx126x 驱动**: EBYTE E22-400MBL 兼容 (433MHz, SF11, BW500kHz, 22dBm)
- **SERIAL3 虚拟串口**: SERIAL3 → LoRaUARTDriver → lora_mavlink → SPI → SX1268
- **hwdef.dat 引脚修复**: SCK/MISO 引脚互换纠正

---

## [v2.1.0] - 2025-12-30

### BNO08x ExternalAHRS 重大修复

- **ENU→NED 坐标转换**: 改用欧拉角中间转换
- **I2C 自动恢复**: 连续 50 次失败后自动重新初始化

---

## [v2.0.0] - 2025-12-30

### BNO08x IMU 稳定运行 & 翼帆控制框架

- **BNO08x**: I2C 设备探测增强，详细调试日志
- **MAVLink**: 修复 MSG_WHEEL_DISTANCE (60) 警告
- **翼帆框架**: 三种类型 (ROTATION/FLAP/FREE)，统一控制接口
- **INA2xx**: 电池监控 I2C 总线支持

---

## [v1.7.0] - 2025-12-29

### MAVLink 带宽优化 (LoRa 适配)

- 禁用 10+ 非关键 MAVLink 消息，带宽从 ~12kbps 降至 ~4.2kbps
- 启用 AP_FENCE 地理围栏

---

## [v1.6.0] - 2025-12-29

### USV 功能精简

- 禁用 11 个不需要的模块 (平衡车/轮编码器/光流/空速计/信标/喷洒器等)
- 保留核心: GPS/罗盘/IMU/电池/MAVLink/Sailboat

---

## [v1.5.0] - 2025-12-29

### 串口映射优化 & Balance Bot 禁用

- SERIAL0=UART0(MAVLink), SERIAL1=UART1(GPS), SERIAL3=LoRa
- 修复 GPS 无法识别问题

---

## [v1.4.0] - 2025-12-28

- u-blox MAX-M10S GPS 数据读取成功
- BNO08x IMU 姿态数据读取成功

---

## [v1.3] - 2025-12-17

- WiFi 配置系统
- LoRa MAVLink 数传 (SX1262)

---

## [v1.2] - 2025-12-15

- BNO08x ExternalAHRS 驱动

---

## [v1.1] - 2025-11-12

- 修复 AP_AHRS API 兼容性和 EKF 函数调用
- 添加 PosHold 模式和 DroneCAN 支持

---

## [v1.0] - 2024-12

- ArduPilot Rover ESP32-S3 IDF 首次移植
