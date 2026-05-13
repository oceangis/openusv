# USV 出水前 Checklist

ArduRover-ESP32S3 V1.0  ·  PCB V2.1  ·  ICM-20948 + AK09916

---

## 0. 当前固件版本指纹

- **Firmware banner**: `ArduRover-ESP32S3 V1.0` (Line 1 + Line 2 board ID)
- **MAVLink autopilot version field**: `flight_sw=0x04070000` (基于 ArduPilot 4.7.0 主干，参数兼容性保留)
- **MCU**: ESP32-S3 N16R8 (PSRAM 8MB)
- **MAC**: D0:CF:13:01:DD:5C
- **构建工具链**: ESP-IDF v5.5.1，编译 binary 1.17 MB（占 app 分区 37%）

---

## 1. 硬件配置（已锁定）

### 主传感器
| 项 | 型号 | 总线 | 地址 | 状态 |
|---|---|---|---|---|
| IMU | ICM-20948 | I2C0 | 0x68 | ✓ 健康 |
| Compass | AK09916（ICM20948 内置） | I2C0 (bypass) | 0x0C | ✓ 健康 |
| GPS | u-blox MAX-M8Q NMEA | UART1 @ 115200 | — | ✓ 已识别（户外才有定位）|

### PWM 输出映射（H10 接口）
| H10 物理 pin | GPIO | hwdef 位 | ArduPilot SERVO# | 用途 |
|:---:|:---:|:---:|:---:|---|
| pin 1 | 45 | 3 | **SERVO3** | **右推进器** (ThrottleRight, FUNCTION=74) |
| pin 2 | 47 | 4 | SERVO4 | 备用 |
| pin 3 | 11 | 6 | SERVO6 | 备用 |
| pin 4 | 48 | 5 | SERVO5 | 备用 |
| pin 5 | 12 | 1 | **SERVO1** | **左推进器** (ThrottleLeft, FUNCTION=73) |
| pin 6 | 13 | 2 | SERVO2 | 备用 |

### 其他外设（来自 IO 分配表 V2.1）
| 功能 | GPIO | 接口 |
|---|---|---|
| LoRa SX1268 SPI | SCK=40, MISO=39, MOSI=41, CS=42, BUSY=2, DIO1=1 | 板载 |
| CAN | TX=20, RX=3, STB=38 | 板载 |
| SD 卡（TF） | SD_DAT0-3, CLK=6, CMD=7 | 板载（**LOG_BACKEND_TYPE=0 已禁用日志**）|
| SBUS RX | GPIO 8（跳线选）| — |
| INA219 电流计 | I2C0 共用 | — |

---

## 2. 已固化的参数修改（重启保留）

### 安全 / Failsafe
```
FS_THR_ENABLE     = 0       # 关闭 RC 失联触发（用 LoRa MAVLink 控制）
LOG_BACKEND_TYPE  = 0       # 关闭 SD 日志（避免 SD 错误阻 prearm）
ARMING_CHECK      = 178     # = Baro(2) + INS(16) + Params(32) + BoardVoltage(128)
                            # 跳过 GPS/RC/Compass/Logging 检查
```

### 速度控制
```
CRUISE_SPEED      = 2.0     # m/s — 默认值，建议出水做 CRUISE_LEARN
CRUISE_THROTTLE   = 50.0    # % — 默认值，同上
WP_SPEED          = 2.0     # m/s — 已对齐 CRUISE_SPEED 避免 base-throttle 外推
MOT_THR_MIN       = 0       # %
MOT_THR_MAX       = 100     # %
ATC_SPEED_P       = 0.2
ATC_SPEED_I       = 0.2
ATC_SPEED_IMAX    = 1.0
```

### 船体类型
```
FRAME_CLASS       = 2       # Boat 水面船
FRAME_TYPE        = 0       # Default — 由 SERVO_FUNCTION 决定实际模式
SERVO1_FUNCTION   = 73      # ThrottleLeft  → H10 pin 5 (GPIO 12)
SERVO3_FUNCTION   = 74      # ThrottleRight → H10 pin 1 (GPIO 45)
                            # → have_skid_steering() 自动激活 = 差速混控
```

### 加速度计 6 面校准（一次性，已完成）
```
INS_ACCOFFS_X     = -0.1355  m/s²
INS_ACCOFFS_Y     = +0.1508  m/s²
INS_ACCOFFS_Z     = +0.1462  m/s²
INS_ACCSCAL_X     = 1.0003   (scale, very close to 1.0)
INS_ACCSCAL_Y     = 0.9980
INS_ACCSCAL_Z     = 0.9991
INS_GYROFFS_X     = -0.0152  rad/s
INS_GYROFFS_Y     = -0.0165  rad/s
INS_GYROFFS_Z     = -0.0097  rad/s
AHRS_TRIM_X       = -0.0061  rad (-0.35°)
AHRS_TRIM_Y       = +0.0116  rad (+0.66°)
```

### EKF
```
AHRS_EKF_TYPE     = 3       # EKF3
EK3_ENABLE        = 1
EK3_SRC1_POSXY    = 3       # GPS
EK3_SRC1_VELXY    = 3       # GPS
EK3_SRC1_POSZ     = 0       # 无气压计，不估高度
EK3_SRC1_YAW      = 1       # Compass
```

---

## 3. 已验证的行为（桌面）

| 测试 | 结果 |
|---|---|
| 启动 + boot banner | ✓ 8.0s, 状态 → MAV_STATE_ACTIVE |
| 22 路遥测 @ 4Hz 60s | ✓ 0 CRITICAL, 0 WARN, EKF flags 稳定 0xa7 |
| IMU 静态偏置 | ✓ \|accel\|=999.3 mg, gyro stdev < 0.25 mrad/s（校准后）|
| 模式切换 MANUAL/HOLD | ✓ 即时确认 |
| MANUAL 模式 ARM + skid steering | ✓ DIFF 范围 ±750μs 全程精确（见 `bench/verify/skid_steering.py` 50Hz 持续 override 测试结果）|
| 差速混控数学 | ✓ 右转 SERVO1↑ SERVO3↓，左转 SERVO1↓ SERVO3↑，原地旋转一推正一推反；中位 DIFF=0 |
| 差速测试评据 | **唯一权威**：`_skid_continuous_override.log`（50Hz 持续 RC override）。不要看 `_verify_skid_arm.log`（单次发送，1.5s 超时，会显示 baseline 偏移 +366μs 的伪象，已废弃）|
| Mission 上传/下载/清除 | ✓ 5 航点 byte-exact 一致（WP0 HOME 覆盖是 ArduPilot 规范行为） |
| 参数读写持久化 | ✓ 跨 3 次重启完全保留，跨 2 次固件重烧也保留 |
| 重启一致性 | ✓ 3 次重启 boot time 8.03s ± 3ms，终态 100% 一致 |
| LoRa MAVLink TX | ✓ 433 MHz SF7 22dBm，初始化成功（已知能收发）|
| CRUISE_LEARN 命令路径 | ✓ MAV_CMD_DO_AUX_FUNCTION 接受，但室内无速度→拒绝学习（正确行为）|
| AUTO 模式 | ✓ 模式可进入，但 ARM 被 AHRS 检查拒绝（需 GPS 锁定）|

---

## 4. 已知限制（设计合理，非 bug）

| 项 | 原因 | 出水后状态 |
|---|---|---|
| 室内 GPS 无定位 | 卫星信号被遮挡 | ✓ 户外露天会锁定 |
| 室内 AHRS 状态 BAD | 没有 GPS → EKF3 退化为 DCM | ✓ 户外 GPS 锁后 EKF3 启动正常 |
| 罗盘磁场过大（1980） | 桌面附近有强磁干扰（USB/笔记本/显示器）| 远离金属/电子设备 + 户外重做 COMPASS_CAL |
| AUTO 模式 ARM 阻拦 | 缺位置估计 | 户外 GPS 锁后 ARM 通过 |
| CRUISE_LEARN 不能启动 | 无速度估计 | 户外有 GPS 后正常工作 |
| ACRO/STEERING 模式不存在 | 固件被 usv-simplifier 精简 | USV 不需要这两个模式 |

---

## 5. 出水当日工作流（建议顺序）

### 步骤 1 — 上电前
- [ ] 双推进器 ESC 信号线确认：**左 ESC → H10 pin 5**，**右 ESC → H10 pin 1**
- [ ] ESC 共阴：所有 ESC GND → H10 GND
- [ ] ESC 信号 VCC：H10 上 5V/3.3V 跳线帽与 ESC 信号要求匹配（多数 ESC 要 5V）
- [ ] 主电池连接、电压在 ESC 工作范围内
- [ ] GPS 天线朝天，避免船体金属遮挡
- [ ] 罗盘 ≥ 30cm 远离电池主电流和电机

### 步骤 2 — 通电首启（船边码头静止）
- [ ] 等启动横幅：`ArduRover-ESP32S3 V1.0`
- [ ] LoRa 链路握手成功
- [ ] 等 GPS 锁定（HEARTBEAT state: CALIBRATING → STANDBY，可能需 30-90 秒）
- [ ] 等 STATUSTEXT: `EKF3 IMU0 tilt alignment complete` + `MAG0 initial yaw alignment complete`
- [ ] **必做：户外 COMPASS_CAL**
  - MAVProxy: `compassmot` 或 `compass cal`
  - 用 MAVLink: `MAV_CMD_DO_START_MAG_CAL`
  - 旋转船 360° 各轴，30-60 秒
  - 保存并重启

### 步骤 3 — 桌面 / 浅水域试验
- [ ] MANUAL 模式 ARM 一次（验证电机响应）
- [ ] 遥控前进、后退、左转、右转、原地旋转
- [ ] 检查无异常震动、漏水、过热

### 步骤 4 — CRUISE_LEARN（开放水域，无浪）
- [ ] MANUAL 模式跑出舒服的稳定巡航速度（30 秒）
- [ ] MAVLink 发: `MAV_CMD_DO_AUX_FUNCTION` param1=50, param2=2 (start)
- [ ] 等 STATUSTEXT: `Cruise Learning started`
- [ ] 持续匀速 ~3 秒
- [ ] 等 STATUSTEXT: `Cruise Learned: Thr:XX Speed:Y.Y`
- [ ] 验证 CRUISE_SPEED / CRUISE_THROTTLE 已自动保存

### 步骤 5 — AUTO 模式首次试航
- [ ] 上传一条 3-5 航点小回路（半径 < 100m）
- [ ] 检查 mission readback 正确
- [ ] 切到 AUTO 模式
- [ ] ARM（这次应通过，因 AHRS 已就绪）
- [ ] **保持 LoRa 遥控在身边**，准备紧急 MANUAL 接管
- [ ] 观察是否能按航点航行 + 速度合理（接近 CRUISE_SPEED）

### 步骤 6 — Failsafe / 应急路径
- [ ] 测试 GCS 失联：让 LoRa 关一会，看船是否进 HOLD 模式
- [ ] 测试电池低压告警（若已配 BMS）
- [ ] 验证 RTL 路径（如启用）

---

## 6. 应急参数（万一出问题）

如果船表现异常，可在岸边通过 MAVLink 改这些紧急回到安全状态：

```
# 紧急停船 — 让两推都到中位
MAV_CMD_COMPONENT_ARM_DISARM param1=0

# 紧急切手动
SET_MODE custom_mode=0 (MANUAL)

# 紧急把船拉回（如果 AUTO 失控）
SET_MODE custom_mode=4 (HOLD) — 仅 SETTING，不动
SET_MODE custom_mode=11 (RTL) — 自动返航起点
```

## 7. 参数备份位置（已纳入 git）

- **`params/baseline_v2.parm`** — 当前 529 个参数完整快照（出厂级配置 + 6 面校准）  
  这是 **唯一** 的 NVS 校准记录。若板子 flash 被擦，没这个文件就要重做完整加速度+罗盘校准。
- `params/usv_minimal.param` — 极简引导参数集（老版本）
- `params/README.md` — 各 .parm 文件的解释 + 恢复流程

**恢复命令** (MAVProxy)：
```
mavproxy.py --master=COM58 --baudrate=115200
> param load params/baseline_v2.parm
```

强烈建议：出水前把 `params/baseline_v2.parm` 拷贝一份到 U 盘 / 云盘 / 另一台电脑作为多重备份。

## 8. 验证脚本位置

| 脚本 | 用途 |
|---|---|
| `bench/verify/skid_steering.py` | **50Hz 持续 RC override 测试差速混控**（权威）|
| `bench/verify/stability.py` | Phase 1-3 综合稳定性测试 |
| `bench/verify/param_rw.py` | 参数读写跨重启持久化测试 |
| `bench/verify/modes.py` | 探测所有 ArduRover 模式可用性 |
| `bench/verify/apply_skid_config.py` | 重新应用 SERVO1=73 / SERVO3=74 |
| `bench/verify/accel_cal_verify.py` | 验证 6 面加速度校准结果 |
| `bench/analyze/tlog_dump.py` | 离线分析 MAVProxy session.tlog |
| `bench/flash/flash_com58.bat` | 编译 + 烧录到 COM58 |
| `bench/flash/mavproxy_bridge.bat` | MAVProxy 透传 + tlog 录制 |

详见 `bench/README.md`。

---

## 9. 下次开发的建议方向（出水后再做）

1. **MOT_SAFE_DISARM = 0** 验证（DISARM 时 ESC 是否输出停止信号 vs 中位）
2. **WPNAV_RADIUS / OVERSHOOT** 调优（默认 WP_RADIUS=2m，船大可能需要更大）
3. **ATC_SPEED_FF** 设置（增加前馈减轻 PID 负担）
4. **风浪扰动模型 / 海流补偿**（如需要可后续从 ArduSub 移植 reefing 类策略）
5. **DroneCAN AHRS 模块**（独立 G474 + ICM20948，让主控甩开 EKF3 — 已有设计草案）
6. **Sailboat 模式**（如有翼帆 USV 需求，源码已就绪 SAIL_TYPE 0/1/2）
