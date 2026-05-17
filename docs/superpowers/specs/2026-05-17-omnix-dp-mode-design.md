# OMNIX 4 推进器矢量控制 + DP 动力定位模式 设计文档

| 字段 | 值 |
|---|---|
| 项目名称 | ModeDP — X 型 4 推进器全驱动船动力定位模式 |
| 文档版本 | v1.0 |
| 创建日期 | 2026-05-17 |
| 代码位置 | `f:\opensource\usv_esp32\ardupilot-master\Rover\` |
| 目标平台 | ESP32-S3,先 SITL 验证后上板 |
| 方案 | 方案 A — ModeDP 自带全驱动位置控制器,不改共享库 |

---

## 1. 背景与目标

### 1.1 需求

为 X 型 4 推进器全驱动(holonomic)无人船新增一个**动力定位(Dynamic
Positioning, DP)模式**:船能同时**锁定 GPS 位置**和**锁定艏向**,靠
前进 + 横移 + 转艏三轴推力顶住风和水流。

### 1.2 约束(用户确认)

- 硬件改装中,**SITL 仿真先行**,验证通过后再上板
- 基于 ArduPilot 内置 `FRAME_TYPE_OMNIX`(4 电机 X 型)电机框架
- DP **同时锁定位置 + 航向**(全驱动的核心价值)
- **新增独立飞行模式**,不改造现有 Loiter
- 遥控器杆**微调目标点**:推杆移动锁定点,松杆锁在新位
- 位置源:当前普通 GPS,代码架构**预留 RTK 升级**(只换更准的输入,逻辑不变)
- **DP 模式只对 `FRAME_TYPE==OMNIX` 开放**,差速船等其它船型一律拒绝进入

### 1.3 非目标

- 不做完整的波浪 passive observer / Kalman 观测器(v1 用死区+低通的轻量版)
- 不改造 Loiter、circle、dock 等现有模式
- 不改共享库 `AR_PosControl`(circle/dock 在用,避免回归)
- 不做硬件实物调试(留到推进器装好后)

---

## 2. 开源调研结论

| 来源 | 结论 |
|---|---|
| ArduPilot 论坛 [Omni Boat 保位+控艏帖](https://discuss.ardupilot.org/t/omni-boat-maintain-position-with-translational-motion-and-orientation-with-yaw/100358) | ArduPilot **无**现成模式能做横移保位+独立控艏;官方建议**自定义模式+内嵌 PID** —— 即本方案 A |
| Fossen《Handbook of Marine Craft Hydrodynamics and Motion Control》(2021) + [MSS 工具箱](https://github.com/cybergalactic/MSS) | DP 标准算法 = 3 自由度 NED 系 PID 控制律;MSS 是 MATLAB,不可移植,仅作验证基准 |
| 标准 Rover Loiter (`mode_loiter.cpp`) | 非全驱动:把艏对准目标点后前进/倒车,**不用横向推力**,对 X 型船是浪费 |
| `AR_PosControl` 库 | 非全驱动:输出"前进速度+转艏速率",`get_desired_lat_accel()` 注释明说 "for reporting only" —— 不能直接用于 DP |

**采用的 DP 控制律(Fossen 3-DOF PID):**

```
τ_body = R(ψ)ᵀ · [ Kp·e + Ki·∫e dt + Kd·ė ]
  e      = [N_err, E_err, ψ_err]ᵀ   NED 系位置 + 航向误差
  R(ψ)   = 艏向旋转矩阵,把 NED 系控制力旋回船体系
  τ_body = [forward, lateral, yaw_moment]  船体系三轴控制量
```

---

## 3. 整体架构

```
ModeDP (Rover/mode_dp.cpp) —— 50 Hz 控制循环
   │
   ├─ _enter(): 门禁(FRAME_TYPE==OMNIX) + 抓取当前 Location/艏向为 DP 目标
   │
   ├─ 杆微调层:  RC 杆 → 速度指令 → 积分进 target_pos / target_yaw
   │             (松杆即停,目标锁在新位置)
   │
   ├─ 位置环 (Fossen 3-DOF PID,本模式自带,不碰 AR_PosControl):
   │     NED 位置误差矢量  ──AC_PID_2D──►  NED 系期望力 (F_N, F_E)
   │     航向误差          ──AC_PID────►  期望转艏力矩
   │
   ├─ 旋转:  R(ψ)ᵀ 把 (F_N,F_E) 旋到船体系 → (forward, lateral)
   │
   └─ 输出:  motors.set_throttle(forward)
             motors.set_lateral(lateral)      ← OMNIX 横移,关键
             motors.set_steering(yaw_moment)
                    │
                    └─ AP_MotorsUGV::output_omni() 4 电机混控 (已有,不改)
```

### 3.1 单元边界

| 单元 | 职责 | 依赖 |
|---|---|---|
| `ModeDP` | 模式生命周期、50Hz 循环调度、门禁、失效保护 | `Mode` 基类 |
| 位置环 | NED 位置误差矢量 → NED 期望力,自带死区 | `AC_PID_2D` |
| 航向环 | 艏向误差 → 转艏力矩,自带死区 | `AC_PID` |
| 杆微调 | RC 输入 → 目标点平移 | `rover.channel_*` |
| 4 电机混控 | forward/lateral/steering → 4 路 PWM | `AP_MotorsUGV::output_omni()`(**已有,不改**) |

**关键点:** 仅新增 `ModeDP` 及其位置/航向环。4 电机 X 型混控复用
ArduPilot 现成 `output_omni()`(`FRAME_TYPE_OMNIX`)。`set_lateral()` 接口
本已存在,DP 模式是第一个真正驱动它的模式。

---

## 4. 模式注册

模式编号:Rover 已占用 0,1,3-12,15,16。新模式 **`DP = 17`**(取
`INITIALISING` 之上的新号,避免冲突)。

| 文件 | 改动 |
|---|---|
| `Rover/mode.h` | `enum Number { … DP = 17 }`;新增 `class ModeDP : public Mode` 声明 |
| `Rover/mode.cpp` | `mode_from_mode_num()` 加 `case 17: ret = &mode_dp; break;` |
| `Rover/Rover.h` | 新增 `ModeDP mode_dp;` 成员 |
| `Rover/Rover.cpp` | 构造列表实例化 `mode_dp` |
| `Rover/mode_dp.cpp` | **新文件** — 模式实现主体 |
| `Rover/Parameters.cpp` / `.h` | 注册 `DP_*` 参数组到 `g2` |
| `Rover/wscript` 或编译列表 | 确保 `mode_dp.cpp` 进编译 |

`ModeDP` 需实现的 `Mode` 虚函数:`_enter()`、`_exit()`、`update()`、
`name4()`("DP")、`mode_number()`、`requires_position()=true`、
`requires_velocity()=true`、`is_autopilot_mode()=true`。

`AP_MotorsUGV` 新增公开取值器 `get_frame_type()`(返回 `_frame_type`),
供门禁判断使用。

---

## 5. 参数设计

用 ArduPilot 标准 PID 对象,子参数(P/I/D/IMAX/滤波)自动生成,放 `g2`:

| 参数 | 类型 | 含义 | 默认值 |
|---|---|---|---|
| `DP_POS_*` | `AC_PID_2D` | 位置环 PID(N/E 共用一组增益) | P=0.20 I=0.05 D=0 IMAX=0.5 |
| `DP_YAW_*` | `AC_PID` | 航向环 PID | P=2.0 I=0.10 D=0 IMAX=0.5 |
| `DP_POS_DEADB` | `AP_Float` | 位置死区(m),误差小于此值不动作 | 1.5 |
| `DP_YAW_DEADB` | `AP_Float` | 航向死区(°) | 5.0 |
| `DP_SPEED_MAX` | `AP_Float` | 最大纠偏 / 杆推满时目标移动速度(m/s) | 1.0 |
| `DP_YAW_RATE` | `AP_Float` | 杆推满时目标艏向转动速率(°/s) | 30.0 |
| `DP_DRIFT_MAX` | `AP_Float` | 杆微调累积目标偏离进入点的上限(m) | 50.0 |
| `DP_OPTIONS` | `AP_Int16` | 位掩码:bit0=允许杆微调;其余预留 | 1 |

**位置环为何用 `AC_PID_2D` 而非两个 `AC_PID`:** 船在水中各向同性,
N/E 用同一组增益。`AC_PID_2D` 是二维矢量 PID(Copter 位置控制在用),
误差矢量整体进、力矢量整体出,参数只需一组。

**`DP_POS_DEADB` 与 RTK 预留:** 默认 1.5m 对应当前普通 GPS(漂移 1-2m);
将来上 RTK 后改成 0.2m 即可,**控制逻辑不变** —— 这就是"预留 RTK"的落地。

---

## 6. 控制循环(`ModeDP::update()`,50 Hz)

每拍执行 6 步:

### ① 读状态
```
pos_now  = ahrs NED 位置(相对 EKF origin, m)   — 经 get_relative_position_NED_origin()
yaw_now  = ahrs.get_yaw()                       (rad)
dt       = rover.G_Dt                           (≈0.02 s)
```

### ② 杆微调目标点(`DP_OPTIONS` bit0 开启时)
```
fwd_stick, lat_stick, yaw_stick ← RC 通道归一化 (-1..+1)
对每个杆做中位死区(消除遥控器中点抖动)
若有效:
   v_cmd_body = (fwd_stick, lat_stick) × DP_SPEED_MAX     船体系速度指令
   v_cmd_ned  = R(yaw_now) · v_cmd_body                   旋到 NED
   target_pos += v_cmd_ned · dt                           积分,挪目标点
   target_yaw  = wrap_PI(target_yaw + yaw_stick × DP_YAW_RATE × dt)
松杆 → 不积分 → target 锁死
目标漂移保护: 若 |target_pos − enter_pos| > DP_DRIFT_MAX,夹回边界
```

### ③ 误差 + 死区(波浪滤波 lite)
```
e_ned = target_pos − pos_now                  NED 位置误差矢量
若 |e_ned| < DP_POS_DEADB:  e_ned = 0,且 pid_pos.reset_I()
e_yaw = wrap_PI(target_yaw − yaw_now)
若 |e_yaw| < radians(DP_YAW_DEADB): e_yaw = 0,且 pid_yaw.reset_I()
```

### ④ Fossen 3-DOF PID
```
F_ned  = pid_pos.update_all(e_ned, dt)   NED 系期望力矢量(2D),输入为 ③ 已死区处理的误差
M_yaw  = pid_yaw.update_all(e_yaw, dt)   期望转艏力矩,输入为 ③ 已死区处理的误差
```
注:`AC_PID_2D` 原生 `update_all(target,measurement,dt)` 内部算误差。为让死区
在 PID 之前生效,实现时直接喂误差 —— 即调用时令 measurement=0、target=e_ned
(误差被 ③ 清零时 PID 输入即为 0)。

### ⑤ 旋转回船体系(DP 控制律核心 R(ψ)ᵀ)
```
forward =  cos(yaw_now)·F_ned.x + sin(yaw_now)·F_ned.y
lateral = -sin(yaw_now)·F_ned.x + cos(yaw_now)·F_ned.y
```

### ⑥ 限幅 + 输出
```
forward, lateral ← 约束到 ±100(对应满推力)
g2.motors.set_throttle(forward × 100)
g2.motors.set_lateral(lateral × 100)        ← OMNIX 横移
g2.motors.set_steering(M_yaw, false)
// AP_MotorsUGV::output_omni() 自动完成 4 电机混控
```

**设计要点:**
- 第 ⑤ 步的 `R(ψ)ᵀ` 把 NED 系控制力换算到船体系,船头一转,
  前进/横移分量自动重分配 → **位置保持与航向保持彻底解耦**
- 杆微调改的是目标(第 ②),不是输出 → "松杆即锁新位"自然成立,无需状态机
- 死区清零误差时同步 `reset_I()`,防积分饱和

---

## 7. 失效保护

| 场景 | DP 行为 | 理由 |
|---|---|---|
| 进入门禁 | `FRAME_TYPE≠OMNIX` → `_enter()` 返回 false,GCS 报 "DP: requires OMNIX frame" | 差速船物理上做不出全驱动 DP |
| ARM 前检查 | `requires_position()=true`、`requires_velocity()=true` → 无 GPS / EKF 不健康则拒绝 ARM | 与 GUIDED/AUTO 一致 |
| 运行中丢 GPS / EKF 失效 | 自动切回 **HOLD**(停桨),GCS 报警 | 无位置估计时停下最安全 |
| RC 信号丢失 | DP **继续保位**,杆微调停止(松杆即锁) | DP 天然优点:丢遥控反而稳定在原地,无需特殊处理 |
| 目标漂移 | 杆微调累积偏离进入点超过 `DP_DRIFT_MAX` 时夹回 | 防手滑把船遛飞 |

---

## 8. SITL 验证方案

### 8.1 现状与新增组件

标准 `SIM_Rover.cpp` 只支持 skid / vectored / omni3mecanum / ackermann,
**没有 OMNIX 4 推进器物理模型**。需新增:

- **SITL OMNIX 全驱动船物理模型**:在 `SIM_Rover.cpp` 加 `omnix` 帧标志
  (`strstr(frame_str,"omnix")`),实现响应 forward + lateral + yaw 三路
  输入的全驱动水面运动 + 水阻尼,并**支持注入恒定水流 + 阵风扰动**
  (DP 验证必须有扰动)。
- 实现前先核查项目历史里的"6 种船型模拟器",若已有全驱动船则复用。

### 8.2 三级验证场景

| 级别 | 场景 | 通过判据 |
|---|---|---|
| 1 静水保位 | 进 DP,无扰动 | 位置漂移 < `DP_POS_DEADB`,航向不漂 |
| 2 抗流保位 | 注入 0.5 m/s 恒定水流 + 阵风 | 稳态位置误差 < 3m,航向误差 < 5°,且 `set_lateral()` 被实际调用(横移推力非零) |
| 3 杆微调 | 模拟 RC 推杆 | 目标点平移,松杆锁新位,满杆移动速度 ≈ `DP_SPEED_MAX` |

### 8.3 自动化

沿用现有 `bench/verify/` 套路,新增 `bench/verify/dp_sitl_test.py`:
起 SITL、切 DP、注扰动、采位置/航向/电机输出,判 PASS/FAIL,产 CSV + 报告。

### 8.4 测试分层

- **单元级:** `R(ψ)ᵀ` 旋转、死区、杆积分等纯函数 → autotest 单测
- **集成级:** 上面三级 SITL 场景
- **硬件级:** 推进器装好后,SITL 参数直接搬上板

---

## 9. 交付物清单

| # | 交付物 | 说明 |
|---|---|---|
| 1 | `Rover/mode_dp.cpp` | ModeDP 实现主体 |
| 2 | `Rover/mode.h` / `mode.cpp` 改动 | 模式声明 + 注册 `DP=17` |
| 3 | `Rover/Rover.h` / `Rover.cpp` 改动 | `mode_dp` 实例化 |
| 4 | `Rover/Parameters.cpp` / `.h` 改动 | `DP_*` 参数组 |
| 5 | `AP_MotorsUGV` 改动 | 新增 `get_frame_type()` 取值器 |
| 6 | `SIM_Rover.cpp` 改动 | OMNIX 全驱动船物理模型 + 扰动注入 |
| 7 | `bench/verify/dp_sitl_test.py` | DP 自动化验证脚本 |
| 8 | autotest 单元测试 | `R(ψ)ᵀ`、死区、杆积分纯函数测试 |
| 9 | 文档 | `docs/MODE_DP.md` 使用说明 + 参数调优指南 |

---

## 10. 实施顺序(建议)

1. SITL OMNIX 物理模型(先有仿真环境才能验证)
2. `AP_MotorsUGV::get_frame_type()` + 模式注册骨架(空 ModeDP 能进能出)
3. `DP_*` 参数组
4. 控制循环 ①③④⑤⑥(先不做杆微调,验证静水/抗流保位)
5. 杆微调 ②
6. 失效保护
7. `dp_sitl_test.py` + 三级验证
8. 文档
