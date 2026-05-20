# Changelog

## 目录 ‧ 总回目

| 回数 | 版本 | 日期 | 回目 |
|:---:|:---:|:---:|:---|
| 第十七回 | [v2.8.0](#v280---2026-05-20) | 2026-05-20 | 立库三度通六法　OMNIX 蟹横渡江河 |
| 第十六回 | [v2.7.0](#v270---2026-05-14) | 2026-05-14 | 三重屏障驱气压　九项金锁固船魂 |
| 第十五回 | [v2.6.0](#v260---2026-05-13) | 2026-05-13 | 易名留参守校准　双推差速辟洪涛 |
| 第十四回 | [v2.5.0](#v250---2026-01-15) | 2026-01-15 | 二版电板归正脉　死环重启复归元 |
| 第十三回 | [v2.4.0](#v240---2026-01-02) | 2026-01-02 | 虚舟翼帆乘风起　流水自检通 CI |
| 第十二回 | [v2.3.0](#v230---2026-01-02) | 2026-01-02 | 双扇误写终归正　重启不再失残篇 |
| 第十一回 | [v2.2.0](#v220---2026-01-02) | 2026-01-02 | 微波长传通遥控　错引脚正归经络 |
| 第十回 | [v2.1.0](#v210---2025-12-30) | 2025-12-30 | ENU 易换归 NED　断路自愈续姿态 |
| 第九回 | [v2.0.0](#v200---2025-12-30) | 2025-12-30 | 稳得姿态 BNO 现　三型翼帆共马辔 |
| 第八回 | [v1.7.0](#v170---2025-12-29) | 2025-12-29 | 削十冗讯节带宽　立围栏护舟安宁 |
| 第七回 | [v1.6.0](#v160---2025-12-29) | 2025-12-29 | 削十一冗留六核　USV 精简归正法 |
| 第六回 | [v1.5.0](#v150---2025-12-29) | 2025-12-29 | 串口重排引 GPS　削平衡车守正道 |
| 第五回 | [v1.4.0](#v140---2025-12-28) | 2025-12-28 | u-blox 锁星明方位　BNO 报姿出三轴 |
| 第四回 | [v1.3](#v13---2025-12-17) | 2025-12-17 | WiFi 近遥通配置　SX1262 立长传 |
| 第三回 | [v1.2](#v12---2025-12-15) | 2025-12-15 | 外接 BNO 立 AHRS　内卸 EKF 减重担 |
| 第二回 | [v1.1](#v11---2025-11-12) | 2025-11-12 | AHRS 接口归正律　PosHold DroneCAN 同立 |
| 第一回 | [v1.0](#v10---2024-12) | 2024-12 | ArduPilot 初登 S3　IDF 起航踏新程 |

---

## [v2.8.0] - 2026-05-20

> **第十七回　立库三度通六法　OMNIX 蟹横渡江河**

### OMNIX 4 推全驱动船全路径模式 + AR_OmniControl 共享 PID 库

X 型 4 推进器全驱动船 (`FRAME_TYPE_OMNIX`) 的 5 阶段路径模式工程,从单一定点的 DP 模式扩展到完整的 mission 跟随、GCS 拖点、定点驻留、回家等全部规划路径模式。差速船代码完全不受影响 —— 所有 OMNIX 分支都是 `if (FRAME_TYPE_OMNIX) { update_omnix*(); return; }` 早返回,差速路径保持不动。

#### 新增共享控制器库 `libraries/AR_OmniControl/`(P1)

3-DOF position + heading PID 控制器,所有 OMNIX 路径模式共用一个 `g2.omni_ctrl` 单例:
- NED 位置 PID(死区抗 GPS 漂)+ 航向 PID(死区)→ `R(ψ)ᵀ` 旋到船体系 → forward / lateral / steering 三路归一化输出
- **PID 内核从原 ModeDP::update() 1:1 移植**(`commit 668791057f` 与 baseline `8bf90e8f0f:mode_dp.cpp:122-185` 字符级匹配),保证行为等价
- 11 个 PID 参数保留 `DP_*` 前缀挂到 g2 subgroup 60(`DP_POS_P/I/D`、`DP_YAW_P/I/D`、`DP_POS_DB`、`DP_YAW_DB`、`DP_SPEED`、`DP_IMAX`、`DP_OPTIONS`),EEPROM 键完全不变 —— 已 tune 的值跨重构无损
- ModeDP 从 ~185 行降到 70 行薄壳:仅入口门禁 + target snapshot + dispatch + 失位降级 HOLD

#### 6 个路径模式全部 OMNIX 化(P2 - P5)

| Mode | Number | OMNIX 阶段 | Yaw 策略 |
|---|:---:|:---:|---|
| DP | 17 | P1 | LOCK_INITIAL(snapshot) |
| AUTO | 10 | P2 | 4 strategies |
| GUIDED — WP submode | 15 | P3 | 4 strategies |
| LOITER | 5 | P4 | LOCK_INITIAL(hard-coded,静止无切线) |
| RTL | 11 | P5 | 4 strategies |
| SMART_RTL — PathFollow | 12 | P5 | 4 strategies |

每个模式遵循统一模式:`_enter()` snapshot 艏向 + reset 控制器 → `update()` 顶部 OMNIX 早返回 → `update_omnix_wp()`(或 `update_omnix()` for Loiter) 取 target_pos NED(从 `g2.wp_nav.get_destination()` 或 mode-specific 来源) → `compute_omnix_target_yaw()` 计算 target_yaw → 喂 `g2.omni_ctrl` → 失位降级 HOLD。

#### 4 种 yaw 策略(`OMNI_YAW_MODE` 参数,P2)

新增 g2 参数 `OMNI_YAW_MODE`(AP_Int8 default 1)+ `OMNI_LOS_LOOK`(AP_Float default 2.0m):

- **`0 LOCK_INITIAL`** —— 进模式时 snapshot 艏向,全程锁定。**蟹形走** —— 船头朝北、横走向东,OMNIX 独有的高级特性
- **`1 TANGENT`** —— 跟 `wp_nav.nav_bearing_cd()` 路径切线,看起来像传统船(默认)
- **`2 POINT_NEXT_WP`** —— 艏向永远指 next waypoint(`wp_bearing_cd`),侦察 / 观察场景
- **`3 MANUAL_RC`** —— 路径自动走,RC 第 4 通道手控艏向(90 deg/s @ full stick),积分到 `_omni_initial_yaw`

#### 共享 helper 重构(P4)

`compute_omnix_target_yaw()` 最初在 ModeAuto + ModeGuided 各 copy 一份(P2 + P3)。**P4 把它抽到 `mode.cpp` 的 file-scope free function** `omnix_compute_target_yaw(OmniYawState&, float dt)`:
- 引入 `OmniYawState` struct(`initial_yaw` + `rc_yaw_integ`)放在 mode.h
- ModeAuto + ModeGuided 各自从 `_omni_initial_yaw + _omni_rc_yaw_integ + compute_omnix_target_yaw` 改用 `OmniYawState _omni_yaw_state + 共享 helper`
- `Rover.h` 加 `friend float omnix_compute_target_yaw(...)`(因为 `g2` 是 Rover 私有)
- **二进制 -288 字节**(`0x121a60 → 0x121940`) —— 去重实测有效

#### ModeDock 永久跳过

`Rover/config.h:148` `#define MODE_DOCK_ENABLED 0` —— precland 模块禁用(USV 用不上飞行器的精确着陆),Dock 模式不存在于 binary。规格 §6 P4 Dock 行明确跳过。

#### Loiter delegate 让 RTL/SmartRTL 白送

ModeRTL `reached_destination` 时 delegate 到 `rover.mode_loiter.update()` —— Loiter 在 P4 已经 OMNIX 化,所以 **RTL 到家 station-keep 自动用 OMNIX 控制律**。ModeSmartRTL 的 `StopAtHome` / `Failure` 状态同样 delegate 到 Loiter,**SmartRTL 也免费拿到 OMNIX 到家站位**。

#### 失位保护(所有 OMNIX 模式)

`AR_OmniControl::get_outputs()` 检测到 `ahrs.get_relative_position_NED_origin_float()` 失败时返回 false。所有 update_omnix*() 收到 false → 停所有桨 + `rover.set_mode(rover.mode_hold, ModeReason::EKF_FAILSAFE)`。**这是 ModeDP v1 没有的特性,P1 重构顺便补上**。

#### 5 个新 bench 验证脚本

| 脚本 | 用途 | GPS 要求 |
|---|---|:---:|
| `omnix_mix_test.py` | 4 推 OMNIX 混控正确性(P1 核心,所有阶段 regression 必跑) | 否 |
| `omni_auto_test.py` | AUTO mission 横移证据 | 是 |
| `omni_guided_test.py` | Guided 拖点横移证据 | 是 |
| `omni_loiter_test.py` | LOITER 站位 sanity | 是 |
| `omni_rtl_test.py` | RTL 回家横移证据(SET_HOME 偏移 10m N + 5m E) | 是 |

SmartRTL 没 bench 脚本 —— 它需要 saved path,bench 上 boat 不动建不起 path,留给水中测试。

#### 硬件回归(板子已烧录 P5 末位 commit `fe1fe71b8b`,COM10)

每个 phase 完成后都重跑 `omnix_mix_test.py` + 检查 EEPROM:
- **`omnix_mix_test.py` 5 次 6/6 PASS**(P1 / P2 / P3 / P4 / P5 各跑一次,混控完全没动)
- **EEPROM 5 次稳定**:11 DP_* + 2 OMNI_* 全部 baseline 值,值与重构前完全一致
- **二进制大小**:P1 之后 0x121208 → P5 之后 0x121d70,净增长 ~3KB(全部新代码 + 1 个 .so 节省了重复代码),62% partition free

#### 设计 + 实施文档

- 设计:`docs/superpowers/specs/2026-05-20-omnix-path-modes-design.md`
- P1 plan:`docs/superpowers/plans/2026-05-20-omnix-path-modes-p1.md`(库 + ModeDP 重构,9 task)
- P2 plan:`docs/superpowers/plans/2026-05-20-omnix-path-modes-p2.md`(ModeAuto + 4 yaw,7 task)
- P3 plan:`docs/superpowers/plans/2026-05-20-omnix-path-modes-p3.md`(ModeGuided WP,7 task)
- P4 plan:`docs/superpowers/plans/2026-05-20-omnix-path-modes-p4.md`(refactor + Loiter,8 task)
- P5 plan:`docs/superpowers/plans/2026-05-20-omnix-path-modes-p5.md`(RTL + SmartRTL,7 task)

#### 不做(规格内决策)

- **ModeDock OMNIX**:precland 在 USV 项目里永久禁用
- **Guided HeadingAndSpeed / TurnRateAndSpeed lateral**:GCS 协议(SET_POSITION_TARGET_LOCAL_NED)的 lateral velocity 位还没实现支持,复杂度大收益小
- **Guided SteeringAndThrottle lateral**:MANUAL_CONTROL 透传协议本身没 lateral field

#### 剩下待办(不在本版本)

- **5 个 GPS-gated 脚本闭环**:`dp_final_test.py` + 4 个 `omni_*_test.py` 全部需要 GPS lock ≥ 8 sats。当前板子在雅加达室内 0 sats,需要搬到窗边或户外才能跑闭环
- **P6 出水验证**(规格 §5):7 步水中测试流程已写好,真下水才能验。建议出水顺序:MANUAL → DP 系泊 → AUTO 矩形 lawnmower(TANGENT)→ AUTO 蟹形巡航(LOCK_INITIAL,OMNIX 独有)→ GUIDED 拖点 → RTL 回家 → LOITER 半小时漂移

---

## [v2.7.0] - 2026-05-14

> **第十六回　三重屏障驱气压　九项金锁固船魂**

### 无气压计 EKF 加固 + 9 个 USV 默认值锁定 + 30 分钟 soak 验证套件

#### baroless EKF — 三层防护（编译时 + 代码默认 + NVS）
- **问题**：GUIDED/AUTO ARM 在 GPS 锁定后仍被 `PreArm: AHRS: EK3 sources require Baro` 拒绝。原因：ArduPilot 在 `AP_NavEKF_Source::pre_arm_check()` 循环检查 3 个 source set（lane switching failover），任何一个 `EK3_SRCx_POSZ = 1 (BARO)` 都触发 `baro_required = true`，而我们的板子没有气压计硬件
- **根因**：PRE_WATER_CHECKLIST 写的 `EK3_SRC1_POSZ = 0` 实际 NVS 是 `1 (BARO)`，文档失实
- **三层防护**：
  1. **编译时**：`hwdef.h:15 AP_BARO_ENABLED = 0`（已有）— AP_Baro library 完全不编进 binary
  2. **代码默认（新增）**：`Rover/Parameters.cpp` 末尾 `AP_Param::set_default_by_name()` 把 `EK3_SRC{1,2,3}_POSZ` 强制设为 `3, 0, 0`（NVS 优先，仅 factory reset / 第一次烧录生效）
  3. **NVS**：用户 PARAM_SET 仍然永远优先
- **验证**：factory reset NVS → 重启 → 读 `EK3_SRC2_POSZ = 0`（来自代码默认覆盖，铁证机制生效）

#### 9 个 USV 关键默认值锁定（`Rover/Parameters.cpp:904-928`）
ESP32 only block，全部用 `set_default_by_name()` 不动 NVS：
- `EK3_SRC1_POSZ = 3 (GPS)`，`EK3_SRC2_POSZ = 0`，`EK3_SRC3_POSZ = 0` — 无气压计 EKF
- `SERVO1_FUNCTION = 73 (ThrottleLeft)`，`SERVO3_FUNCTION = 74 (ThrottleRight)` — 差速船
- `FRAME_CLASS = 2 (Boat)` — 车型
- `ARMING_CHECK = 178` — 跳过 GPS/RC/Compass/Logging
- `FS_THR_ENABLE = 0` — 关 RC 失联（用 LoRa）
- `LOG_BACKEND_TYPE = 0` — 关 SD 日志
- **意义**：任何时候 EEPROM 损坏 / `param reset_all` 后，板子重启依然是可用的差速 USV，不会变成"装错船型的危险默认 Rover"

#### 30 分钟桌面 soak 验证（零泄漏基线）
- **静默 baseline** (`bench/verify/long_soak.py`)：USB 供电、无 ESC、无 SBUS、纯监听
  - **free heap 8,257,536 → 8,257,536 字节，零漂移**
  - EKF flags 0x033F 全程不抖、HB/RC/IMU 速率全稳、0 STATUSTEXT 警告
- **Work-load** (`bench/verify/workload_soak.py`)：MANUAL+ARMED + 50 Hz RC 中位 override
  - RC override 投递 **89,975 / 90,004 = 100.0%, 0 errors**
  - 伺服 s1=1500、s3=1500 **range=0**（精确锁死）
  - **free heap 仍 0 漂移**
- 这两组 soak 一起证明 SBUS RMT 迁移（commit b09be0bb00）+ baroless 修复没有引入任何泄漏

#### GPS 锁 + EKF 健康度验证（雅加达桌面）
- **u-blox MAX-M8Q FIX_3D**，22-26 颗卫星，HDOP 0.8
- 60 秒静态位置漂移 **0.02 m**（5 cm 量级，u-blox 标称水平）
- EKF3 variances：`vel=0.016`，`pos_h=0.020`，`compass=0.014`（全 ≪ 0.1，excellent）
- GPS_RAW_INT / GLOBAL_POSITION_INT 都稳定 5 Hz

#### GUIDED/AUTO ARM 桌面验证（在 baroless fix 之后）
- GUIDED ARM：`MAV_RESULT_ACCEPTED`，伺服保持中位（没收到位置命令时）
- AUTO ARM + 3 航点 mission：`MAV_RESULT_ACCEPTED`，伺服**立即驱动**（s1=1900 全油门 + s3=1267 转向，差速混控指向第一个航点）
- ⚠ 这意味着出水时 AUTO ARM 后船立刻冲向第一个航点 — `PRE_WATER_CHECKLIST §5` 已加入严重警告

#### 8 个新验证脚本（`bench/verify/`）
| 脚本 | 用途 |
|---|---|
| `long_soak.py` | 30 分钟纯静默 soak（baseline 泄漏检测）|
| `workload_soak.py` | 30 分钟工况 soak（ARM + 50Hz RC override）|
| `gps_check.py` | 60 秒 GPS 状态快查 |
| `auto_arm_verify.py` | GUIDED/AUTO ARM 验证 + mission 自启动观察 |
| `param_persist_check.py` | 重启参数持久化 + dump 完整 baseline.parm |
| `factory_reset_verify.py` | NVS 擦除验证代码层默认值机制 |
| `_restore_baseline.py` | 从 baseline.parm 批量 PARAM_SET 恢复 |
| `quick_regression.py` | 2 分钟改后烟雾测试（之前已存在，未追踪）|

所有脚本强制 `sys.stdout.reconfigure(encoding="utf-8")` 防 Windows GBK 控制台在长 soak 中遇 µ/✓ 字符崩溃。

#### 533 参数完整快照 (`params/baseline_v3.parm`)
- 包括加速度 6 面校准、AHRS_TRIM、差速混控配置、LoRa SF7 配置、新的 baroless EKF 源
- 注意：`INS_GYROFFS_*` 备份了也没用 — ArduPilot 每次启动自动 init_gyro 并写覆盖 NVS（`INS_GYR_CAL=1` 默认）
- 真正"独家信息"是 INS_ACCOFFS/SCAL/ID + COMPASS_OFS/DIA/ODI/DEV_ID + AHRS_TRIM_X/Y

#### 文档更新
- `PRE_WATER_CHECKLIST.md §2` — EKF 参数列表对齐到现实值
- `PRE_WATER_CHECKLIST.md §2.1`（新增）— 9 个代码层兜底默认值清单 + 意义
- `PRE_WATER_CHECKLIST.md §3` — 加入 GUIDED/AUTO ARM 验证记录
- `PRE_WATER_CHECKLIST.md §4` — 移除"AUTO ARM 阻拦"（已解决）
- `PRE_WATER_CHECKLIST.md §5` — 加入 AUTO ARM 即启推进器严重警告 + 分步骤安全流程
- `PRE_WATER_CHECKLIST.md §8` — 8 个新脚本加入工具清单

---

## [v2.6.0] - 2026-05-13

> **第十五回　易名留参守校准　双推差速辟洪涛**

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

> **第十四回　二版电板归正脉　死环重启复归元**

### PCB V2.1 适配
- 修复重启死循环及硬件引脚配置

---

## [v2.4.0] - 2026-01-02

> **第十三回　虚舟翼帆乘风起　流水自检通 CI**

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

> **第十二回　双扇误写终归正　重启不再失残篇**

### 修复参数持久化关键BUG

- **问题**: AP_FlashStorage::init() 从 Sector 1 加载数据后，current_sector 仍为 0
- **后果**: 新参数写入 Sector 0，重启后加载 Sector 1 旧数据，导致参数丢失
- **修复**: 在 load_sector() 循环中记录 IN_USE 的 sector，加载完成后正确设置 current_sector
- 显式初始化构造函数成员变量，避免未定义初始值

---

## [v2.2.0] - 2026-01-02

> **第十一回　微波长传通遥控　错引脚正归经络**

### LoRa MAVLink 数传初步完成

- **lora_mavlink 组件**: TX/RX 实现 (环形缓冲区 + FreeRTOS 任务)
- **sx126x 驱动**: EBYTE E22-400MBL 兼容 (433MHz, SF11, BW500kHz, 22dBm)
- **SERIAL3 虚拟串口**: SERIAL3 → LoRaUARTDriver → lora_mavlink → SPI → SX1268
- **hwdef.dat 引脚修复**: SCK/MISO 引脚互换纠正

---

## [v2.1.0] - 2025-12-30

> **第十回　ENU 易换归 NED　断路自愈续姿态**

### BNO08x ExternalAHRS 重大修复

- **ENU→NED 坐标转换**: 改用欧拉角中间转换
- **I2C 自动恢复**: 连续 50 次失败后自动重新初始化

---

## [v2.0.0] - 2025-12-30

> **第九回　稳得姿态 BNO 现　三型翼帆共马辔**

### BNO08x IMU 稳定运行 & 翼帆控制框架

- **BNO08x**: I2C 设备探测增强，详细调试日志
- **MAVLink**: 修复 MSG_WHEEL_DISTANCE (60) 警告
- **翼帆框架**: 三种类型 (ROTATION/FLAP/FREE)，统一控制接口
- **INA2xx**: 电池监控 I2C 总线支持

---

## [v1.7.0] - 2025-12-29

> **第八回　削十冗讯节带宽　立围栏护舟安宁**

### MAVLink 带宽优化 (LoRa 适配)

- 禁用 10+ 非关键 MAVLink 消息，带宽从 ~12kbps 降至 ~4.2kbps
- 启用 AP_FENCE 地理围栏

---

## [v1.6.0] - 2025-12-29

> **第七回　削十一冗留六核　USV 精简归正法**

### USV 功能精简

- 禁用 11 个不需要的模块 (平衡车/轮编码器/光流/空速计/信标/喷洒器等)
- 保留核心: GPS/罗盘/IMU/电池/MAVLink/Sailboat

---

## [v1.5.0] - 2025-12-29

> **第六回　串口重排引 GPS　削平衡车守正道**

### 串口映射优化 & Balance Bot 禁用

- SERIAL0=UART0(MAVLink), SERIAL1=UART1(GPS), SERIAL3=LoRa
- 修复 GPS 无法识别问题

---

## [v1.4.0] - 2025-12-28

> **第五回　u-blox 锁星明方位　BNO 报姿出三轴**

- u-blox MAX-M10S GPS 数据读取成功
- BNO08x IMU 姿态数据读取成功

---

## [v1.3] - 2025-12-17

> **第四回　WiFi 近遥通配置　SX1262 立长传**

- WiFi 配置系统
- LoRa MAVLink 数传 (SX1262)

---

## [v1.2] - 2025-12-15

> **第三回　外接 BNO 立 AHRS　内卸 EKF 减重担**

- BNO08x ExternalAHRS 驱动

---

## [v1.1] - 2025-11-12

> **第二回　AHRS 接口归正律　PosHold DroneCAN 同立**

- 修复 AP_AHRS API 兼容性和 EKF 函数调用
- 添加 PosHold 模式和 DroneCAN 支持

---

## [v1.0] - 2024-12

> **第一回　ArduPilot 初登 S3　IDF 起航踏新程**

- ArduPilot Rover ESP32-S3 IDF 首次移植
