# OMNIX 路径模式族设计 — AR_OmniControl + 各路径模式 OMNIX 分支

**日期**: 2026-05-20
**项目**: esp32s3rover / ardupilot_rover_esp32s3_idf
**前置**: `2026-05-17-omnix-dp-mode-design.md`（ModeDP 已实现并桌面验证）

## 目标

让 X 型 4 推进器全驱动船（`FRAME_TYPE_OMNIX`）在所有现有"路径跟踪类"模式下都能利用横移自由度，而不是降级当差速船跑。

涉及模式：**AUTO、GUIDED、LOITER、RTL、SMART_RTL、DOCK**。

差速船配置完全不受影响——每个模式都是"OMNIX 分支 + 原差速分支"。

## 现状

| 模式 | 当前是否调 `set_lateral` | OMNIX 利用率 |
|------|---|---|
| MANUAL | ✅ | 100% |
| POSHOLD | ✅ | 部分（仅站位） |
| DP (17) | ✅ | 100% — Fossen 3-DOF |
| AUTO / GUIDED / RTL / SMART_RTL / LOITER / DOCK | ❌ | 0% — 仅 throttle+steering |

`AR_WPNav` 和 `AR_AttitudeControl` 都不感知 `FRAME_TYPE_OMNIX`，输出层只给 `throttle/steering`。`AP_L1_Control` 在 libraries 里但 Rover 未使用。

## 架构

### 1. 共享控制器 — `AR_OmniControl`

新建 `libraries/AR_OmniControl/`。把 `Rover/mode_dp.cpp` 已经验证过的 PID 内核抽出来，所有 OMNIX 路径模式共用同一个 3-DOF 控制器。

```cpp
class AR_OmniControl {
public:
    static const struct AP_Param::GroupInfo var_info[];
    AR_OmniControl();

    // 设置目标(NED 系位置 + 艏向 rad,可选速度前馈)
    void set_target(const Vector2f& pos_ned,
                    float yaw_rad,
                    const Vector2f& vel_ff_ned = Vector2f{0,0});

    // 主循环:读 AHRS → 算 PID → 旋到船体系
    void update(float dt);

    // 取输出:船体系 forward / lateral / steering(归一化 -1..+1)
    // 返回 false 表示位置估计不可用 → 调用方应转 HOLD
    bool get_outputs(float& fwd, float& lat, float& steer_norm);

    // 切模式时清积分器
    void reset();

private:
    // PID 增益 + 死区(自有参数,见下方"参数归宿")
    AP_Float _pos_p, _pos_i, _pos_d;
    AP_Float _yaw_p, _yaw_i, _yaw_d;
    AP_Float _pos_db, _yaw_db;
    AP_Float _speed_max, _imax;

    Vector2f _target_pos;
    float    _target_yaw;
    Vector2f _vel_ff;
    Vector2f _pos_integ;
    Vector2f _prev_pos_err;
    float    _yaw_integ;
    float    _prev_yaw_err;
    float    _out_fwd, _out_lat, _out_steer;
    bool     _outputs_valid;
};
```

**参数归宿（关键决策，解决"谁拥有 DP_\*"问题）**：

- `AR_OmniControl` 自带 `var_info` 拥有 PID 参数。
- 在 `Parameters.h` 的 `g2` 里实例化为 `AR_OmniControl omni_ctrl`，对外暴露为 `g2.omni_ctrl`。
- **参数名保留 `DP_*` 前缀不变**——靠 `g2` 里的 `AP_SUBGROUPINFO(omni_ctrl, "DP_", ..., ParametersG2, AR_OmniControl)` 把 `g2.omni_ctrl` 的子组前缀设为 "DP_"。EEPROM 键稳定不变。
- `ModeDP::var_info` 删掉 PID 参数（移到 `AR_OmniControl`），保留 mode-specific 标志（如 `DP_OPTIONS`）。EEPROM 中已存的 `DP_POS_P` 等通过新的 AP_Param 路径解析到同一存储位置（参数名不变 → token 不变）。
- 所有 OMNIX 路径模式都用 `g2.omni_ctrl`（单例），保证一套参数全模式共享。

**`OMNI_YAW_MODE`、`OMNI_LOS_LOOK`** 作为新参数放在 `g2`（不在 `AR_OmniControl` 内部，因为艏向策略是模式层逻辑，控制器只接收 `target_yaw`）。

控制律（搬运自 ModeDP，未来再分迭代）：

```
e_pos = target_pos - pos_now           # NED
if |e_pos| < pos_db: e_pos = 0; integ = 0
f_ned = Kp*e_pos + Ki*∫e_pos + Kd*ė + Kv*vel_ff
e_yaw = wrap_PI(target_yaw - yaw_now)
if |e_yaw| < yaw_db: e_yaw = 0
m_yaw = Kp_y*e_yaw + Ki_y*∫e_yaw + Kd_y*ė_yaw

# R(ψ)^T:把 NED 力旋到船体系
forward = cos(ψ)*f_ned.x + sin(ψ)*f_ned.y
lateral = -sin(ψ)*f_ned.x + cos(ψ)*f_ned.y

# 限幅
forward, lateral, m_yaw ∈ [-1, +1]
```

参数复用现有 `DP_*` 组——已经 bench 验证过、tune 过，避免双套配置漂移。后续如要细分（例如 LOITER 想更软），再追加 `OMNI_<mode>_*` 覆盖。

### 2. ModeDP 重构为薄壳（P1）

把 `Rover/mode_dp.cpp` 的 PID 实现搬到 `AR_OmniControl`。`ModeDP` 保留：
- `var_info`（`DP_*` 参数定义）
- 入口门禁（`FRAME_TYPE_OMNIX` + 位置有效性）
- 目标抓取（进入时 snapshot 当前位置+艏向）
- 转发到 `AR_OmniControl::update()` 和 `get_outputs()`

行为不变。回归用 `bench/verify/dp_*.py` 一把过。

### 3. 各路径模式分支

每个 OMNIX-aware 模式在 `update()` 顶端加：

```cpp
void ModeAuto::update() {
    if (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX) {
        update_omnix();
        return;
    }
    // —— 原差速逻辑保持不变 ——
}
```

`update_omnix()` 的工作（各模式共性）：
1. 取目标位置（mode-specific，见下表）
2. 按 `OMNI_YAW_MODE` 算目标艏向
3. 喂给 `AR_OmniControl`
4. `get_outputs()` → `set_throttle/set_lateral/set_steering`
5. 位置无效 → 触发模式降级到 HOLD

#### 各模式目标来源

| 模式 | target_pos 来源 | vel_ff 来源 | 默认 yaw 策略 |
|------|---------|---------|---------|
| **AUTO** | `g2.wp_nav.get_target_location()` 转 NED | `wp_nav.get_desired_velocity_NED()` | 跟切线 |
| **GUIDED** | GCS `SET_POSITION_TARGET_*`（已存在 `_guided_position`） | GCS 提供 or 0 | 见下规则 |
| **LOITER** | 进入点（snapshot） | 0 | 锁初始 |
| **RTL** | home / rally（沿用现有 wp_nav） | wp_nav 速度 | 跟切线，到家后锁定 |
| **SMART_RTL** | `g2.smart_rtl.thorough_cleanup_path()` 当前点 | wp_nav 速度 | 跟切线 |
| **DOCK** | dock target（沿用现有 `_dock_pos`） | 0 | 对准 dock 朝向 |

#### 艏向策略实现

全局参数 `OMNI_YAW_MODE`：

- `0` **LOCK_INITIAL** — 进模式那一刻的艏向，蟹形走（OMNIX 独有）
- `1` **TANGENT** — 目标艏向 = atan2(target_pos − pos_now)（默认）
- `2` **POINT_NEXT_WP** — AUTO 时指向 next waypoint；其他模式 fallback 到 TANGENT
- `3` **MANUAL_RC** — 路径自动走，RC 第 4 通道控艏向（CH4 → target_yaw rate 积分）

新增辅助参数：
- `OMNI_LOS_LOOK`（前视距离 m，TANGENT 模式下计算切线时用，default 2.0）

**GUIDED 艏向优先级规则**：GCS 在 `SET_POSITION_TARGET_*` 报文里若设置了 `yaw` 字段（即 `type_mask` 未屏蔽 yaw 位），以 GCS 给的为准；否则按 `OMNI_YAW_MODE` 走。

### 4. 失效保护

`AR_OmniControl::get_outputs()` 返回 false 的条件：
- `ahrs.get_relative_position_NED_origin_float()` 失败
- 位置 NaN / inf

调用方（路径模式）收到 false 时的统一动作：
```cpp
g2.motors.set_throttle(0);
g2.motors.set_lateral(0);
g2.motors.set_steering(0);
rover.set_mode(rover.mode_hold, ModeReason::EKF_FAILSAFE);
```

ModeDP v1 没做失位降级，这次一并补上（ModeDP 走薄壳后自动获得）。

## 实施分阶段

| 阶段 | 内容 | 风险 | 验证 |
|------|---|---|---|
| **P1** | 建 `AR_OmniControl` 库；ModeDP 重构为薄壳 | 低 | 现有 `dp_*.py` 全 PASS |
| **P2** | ModeAuto OMNIX 分支 + `OMNI_YAW_MODE` 实现 | 中 | 新增 `omni_auto_test.py` |
| **P3** | ModeGuided OMNIX 分支 | 低 | 新增 `omni_guided_test.py` |
| **P4** | ModeLoiter + ModeDock OMNIX 分支 | 低 | `omni_loiter_test.py`、`omni_dock_test.py` |
| **P5** | ModeRTL + ModeSmartRTL OMNIX 分支 | 低 | `omni_rtl_test.py`、`omni_smart_rtl_test.py` |
| **P6** | 出水验证脚本 + 整合测试 | 低 | 浴缸 / 系泊 / 出水 |

每阶段独立可烧录，差速船不受影响（FRAME_TYPE≠2 走原路径）。

## 验证策略

### 桌面（每阶段）

每个 OMNIX 路径模式一份 `bench/verify/omni_<mode>_test.py`：
1. 切模式成功（OMNIX 配置下）
2. 注入虚拟目标 / 推动 GPS（用 `MAV_CMD_DO_REPOSITION` 或 wp upload）
3. 看 SERVO_OUTPUT_RAW 出现 4 路非中立输出
4. 切 `OMNI_YAW_MODE` 0/1/2 → 验证艏向响应一致性
5. 注入 EKF failsafe → 验证降级到 HOLD

### 浴缸/系泊

绑绳测试：
- AUTO：上传 4 个 waypoint 矩形 → 看船是不是真走矩形
- GUIDED：拖点漂移 → 看船是不是跟踪
- 三种 yaw mode 各跑一次

### 出水

- 直线 50m（AUTO，TANGENT yaw）
- 矩形 lawnmower 30×20m（AUTO，TANGENT yaw）
- 蟹形横移：船头锁北、横走 20m（AUTO，LOCK_INITIAL yaw）
- RTL 三种 yaw mode 各一次
- DOCK 进坞（系泊端）

## 不做（YAGNI）

- 不动 `AR_WPNav` 内部（保持差速船兼容）
- 不引入 `AP_L1_Control`（OMNIX 有横移直接消除 cross-track，不需要 L1 几何）
- 不做 OMNIX 专属轨迹生成（沿用 AR_WPNav 的 S-curve + spline）
- 不重命名 `DP_*` 参数为 `OMNI_*`（避免迁移成本和参数漂移）
- 不做 stick-nudge（路径模式下不允许杆微调，要微调切 MANUAL）

## 风险与缓解

| 风险 | 缓解 |
|---|---|
| `AR_WPNav` 期望差速船反馈（throttle/steering），OMNIX 旁路后可能干扰 wp 切换逻辑 | 仅旁路输出层，保留 wp_nav.update() 调用；wp 切换判定继续依赖 wp_nav 内部距离/到达判定 |
| ModeDP 重构引入回归 | P1 完成后立即跑全套 `dp_*.py` 回归；不通过不进 P2 |
| EKF failsafe 与模式降级的事件竞争 | 用 `ModeReason::EKF_FAILSAFE` 走标准降级路径，不自己处理 |
| 出水时 OMNIX 调参偏离 ModeDP 单独定点的最佳值 | P6 阶段补 `OMNI_<mode>_*` 覆盖参数（在 spec 范围外，下迭代） |

## 关键文件清单

**新建：**
- `libraries/AR_OmniControl/AR_OmniControl.cpp`
- `libraries/AR_OmniControl/AR_OmniControl.h`
- `libraries/AR_OmniControl/CMakeLists.txt`
- `bench/verify/omni_auto_test.py`、`omni_guided_test.py`、`omni_loiter_test.py`、`omni_rtl_test.py`、`omni_smart_rtl_test.py`、`omni_dock_test.py`

`update_omnix()` 作为 private 方法**加在各模式现有的 .cpp 文件里**（`mode_auto.cpp`、`mode_guided.cpp` 等），不另开新文件——减少文件数量、保持每个模式所有逻辑同处。

**修改：**
- `Rover/mode_dp.cpp`（重构为薄壳）
- `Rover/mode.cpp`、`mode.h`（艏向参数声明、ModeXXX::update_omnix 声明）
- `Rover/Parameters.cpp`、`Parameters.h`（新增 `OMNI_YAW_MODE`、`OMNI_LOS_LOOK`）
- `components/ardupilot/CMakeLists.txt`（touch 触发 CMake 重扫，新增 .cpp 自动被 `GLOB_RECURSE` 收）

**不动：**
- `libraries/AR_WPNav/`、`libraries/AR_AttitudeControl/`、`libraries/AP_L1_Control/`
- ardupilot-master 主线（一切都在项目树）

## 决策记录

- **PID 参数迁移到 `AR_OmniControl`、前缀仍叫 `DP_`**：参数名不变意味着 EEPROM 键稳定，已 tune 好的值不丢；所有 OMNIX 路径模式共享一套，避免配置漂移。
- **默认 `OMNI_YAW_MODE=1`(TANGENT)**：最符合传统船型行为期望；LOCK_INITIAL 蟹形走作为高级特性可选。
- **失位 → HOLD**：统一降级路径，避免每模式各写一套。
- **不复用 AP_L1_Control**：L1 是为非全驱设计的横向加速度控制律；OMNIX 横移自由度让 cross-track 直接可解，无需 L1 几何。
- **阶段顺序 AUTO → GUIDED → LOITER/DOCK → RTL/SMART_RTL**：按"出水紧迫度"排序，AUTO 是出水主用例。
