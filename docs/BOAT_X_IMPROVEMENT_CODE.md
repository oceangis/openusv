# X型船 ArduSub风格改进代码

## 文档信息
- **创建日期**: 2025-11-09
- **目标**: 修正 FRAME_TYPE_BOAT_VECTORED_X 的 lateral 因子
- **参考**: ArduSub VECTORED 配置逻辑

---

## 修改文件
**文件路径**: `libraries/AR_Motors/AP_MotorsUGV.cpp`
**修改位置**: 行704-739 `FRAME_TYPE_BOAT_VECTORED_X` case

---

## 完整修改代码

### 修改前（当前代码）

```cpp
case FRAME_TYPE_BOAT_VECTORED_X:
    /*
     * X-Configuration for boats
     * Layout:
     *      Bow ↑
     *
     *   M1 ↗  ↖ M2    (45° angles)
     *      \  /
     *       \/
     *       /\
     *   M3 ↙  ↘ M4
     *
     * Capabilities:
     * - Omnidirectional movement
     * - Rotate in place
     * - Diagonal motion most efficient
     * - Redundant (works with any 3 thrusters)
     */
    _motors_num = 4;

    const float cos45 = 0.70710678f;  // cos(45°)

    // Motor 1: Left-front 45°
    add_omni_motor(0, cos45, -cos45, -cos45);  // ← 问题：lateral=-cos45

    // Motor 2: Right-front 45°
    add_omni_motor(1, cos45, cos45, cos45);    // ← 问题：lateral=+cos45

    // Motor 3: Left-rear 45°
    add_omni_motor(2, -cos45, -cos45, cos45);  // ← 问题：lateral=+cos45

    // Motor 4: Right-rear 45°
    add_omni_motor(3, -cos45, cos45, -cos45);  // ← 问题：lateral=-cos45

    gcs().send_text(MAV_SEVERITY_INFO, "Motors: Boat X-config (4 thrusters)");
    break;
```

**问题分析**：
- Lateral右移时：M1推（-），M2拉（+），M3拉（+），M4推（-）
- 导致：左前推+右前拉=横移+偏航（错误！）
- 应该：左侧(M1+M3)推，右侧(M2+M4)拉=纯横移

---

### 修改后（推荐代码）

```cpp
case FRAME_TYPE_BOAT_VECTORED_X:
    /*
     * X-Configuration for boats (ArduSub VECTORED style)
     *
     * Layout:
     *      Bow ↑
     *
     *   M1 ↗  ↖ M2    (45° angles)
     *      \  /
     *       \/
     *       /\
     *   M3 ↙  ↘ M4
     *
     * Control Mapping (Rover → ArduSub):
     * ┌──────────────┬────────────────────┬───────────────────────┐
     * │ Rover Input  │ ArduSub Equivalent │ Physical Meaning      │
     * ├──────────────┼────────────────────┼───────────────────────┤
     * │ throttle     │ forward_thrust     │ 前后运动 (Forward)    │
     * │ steering     │ yaw_thrust         │ 偏航旋转 (Yaw)        │
     * │ lateral      │ lateral_thrust     │ 横向平移 (Lateral)    │
     * └──────────────┴────────────────────┴───────────────────────┘
     *
     * Motor Correspondence (Rover → ArduSub VECTORED):
     * ┌───────┬───────┬──────────────────┬──────────────────────┐
     * │ Rover │ 位置  │ ArduSub对应      │ 因子 (T/S/L)         │
     * ├───────┼───────┼──────────────────┼──────────────────────┤
     * │ M1    │ 左前  │ M3 (Port-Fwd)    │ +0.707/-0.707/+0.707 │
     * │ M2    │ 右前  │ M4 (Stbd-Fwd)    │ +0.707/+0.707/-0.707 │
     * │ M3    │ 左后  │ M1 (Port-Aft)    │ -0.707/-0.707/+0.707 │
     * │ M4    │ 右后  │ M2 (Stbd-Aft)    │ -0.707/+0.707/-0.707 │
     * └───────┴───────┴──────────────────┴──────────────────────┘
     *
     * Factor Explanation (add_omni_motor params: throttle, steering, lateral):
     *
     * 1. Throttle Factor (Forward/Backward movement):
     *    - Front motors (M1, M2): +0.707 (push forward)
     *    - Rear motors (M3, M4): -0.707 (pull backward)
     *    - Physics: Differential thrust along longitudinal axis
     *
     * 2. Steering Factor (Yaw rotation):
     *    - Left motors (M1, M3): -0.707 (rotate CCW)
     *    - Right motors (M2, M4): +0.707 (rotate CW)
     *    - Physics: Differential thrust creates torque
     *
     * 3. Lateral Factor (Sideways translation) - FIXED:
     *    - Left motors (M1, M3): +0.707 (push left → move right)
     *    - Right motors (M2, M4): -0.707 (pull right → move right)
     *    - Physics: Pure lateral thrust with zero yaw torque
     *
     * Physical Behavior Examples:
     * ┌──────────────────┬───────────────────────────────────────┐
     * │ Command          │ Motor Output (M1, M2, M3, M4)         │
     * ├──────────────────┼───────────────────────────────────────┤
     * │ Forward 100%     │ (+70.7, +70.7, -70.7, -70.7)          │
     * │ Yaw CW 100%      │ (-70.7, +70.7, -70.7, +70.7)          │
     * │ Lateral Right    │ (+70.7, -70.7, +70.7, -70.7) ← FIXED  │
     * │ Fwd+Yaw          │ (0, +141→100, -141→-100, 0)           │
     * │ Fwd+Lateral      │ (+141→100, 0, 0, -141→-100)           │
     * └──────────────────┴───────────────────────────────────────┘
     *
     * Capabilities:
     * - Omnidirectional movement (full 2D+Yaw control)
     * - Rotate in place (zero-radius turn)
     * - Pure lateral translation (crab walk)
     * - Diagonal motion (most efficient)
     * - Redundant (works with any 3 thrusters)
     *
     * Design Notes:
     * - cos(45°) = 0.70710678 represents thrust projection
     * - ±1 normalization happens in output_omni() function
     * - ArduSub uses ±1.0 factors, Rover uses ±0.707 (equivalent)
     */
    _motors_num = 4;

    const float cos45 = 0.70710678f;  // cos(45°) = 1/√2

    // Motor 1: Left-front 45° (Port-Forward)
    // add_omni_motor(motor_num, throttle_factor, steering_factor, lateral_factor)
    // Corresponds to ArduSub VECTORED M3 (Forward=+1, Yaw=-1, Lateral=+1)
    add_omni_motor(0, cos45, -cos45, cos45);  // ← FIXED: lateral=+cos45

    // Motor 2: Right-front 45° (Starboard-Forward)
    // Corresponds to ArduSub VECTORED M4 (Forward=+1, Yaw=+1, Lateral=-1)
    add_omni_motor(1, cos45, cos45, -cos45);  // ← FIXED: lateral=-cos45

    // Motor 3: Left-rear 45° (Port-Aft)
    // Corresponds to ArduSub VECTORED M1 (Forward=-1, Yaw=+1, Lateral=+1)
    add_omni_motor(2, -cos45, -cos45, cos45); // ← UNCHANGED: lateral=+cos45

    // Motor 4: Right-rear 45° (Starboard-Aft)
    // Corresponds to ArduSub VECTORED M2 (Forward=-1, Yaw=-1, Lateral=-1)
    add_omni_motor(3, -cos45, cos45, -cos45); // ← UNCHANGED: lateral=-cos45

    gcs().send_text(MAV_SEVERITY_INFO, "Motors: Boat X-config (ArduSub style, 4 thrusters)");
    break;
```

---

## 修改总结

### 变化对比表

| Motor | 位置 | 参数位置 | 修改前 | 修改后 | 变化 |
|-------|------|----------|--------|--------|------|
| M1    | 左前 | lateral  | -cos45 | **+cos45** | 符号翻转 |
| M2    | 右前 | lateral  | +cos45 | **-cos45** | 符号翻转 |
| M3    | 左后 | lateral  | +cos45 | +cos45 | 不变 |
| M4    | 右后 | lateral  | -cos45 | -cos45 | 不变 |

### 修改的行数
- **仅修改2行**：M1和M2的lateral因子
- **不影响**：throttle和steering因子
- **不影响**：M3和M4的任何因子

---

## 验证测试

### Test 1: Lateral Right Movement
**命令**: `throttle=0, steering=0, lateral=+100`

**修改前输出**:
```
M1: 0 + 0 + 100*(-0.707) = -70.7%  ← 左前推
M2: 0 + 0 + 100*(+0.707) = +70.7%  ← 右前拉
M3: 0 + 0 + 100*(+0.707) = +70.7%  ← 左后拉
M4: 0 + 0 + 100*(-0.707) = -70.7%  ← 右后推
```
**问题**: 形成对角线推拉 → 横移+旋转！

**修改后输出**:
```
M1: 0 + 0 + 100*(+0.707) = +70.7%  ← 左前推
M2: 0 + 0 + 100*(-0.707) = -70.7%  ← 右前拉
M3: 0 + 0 + 100*(+0.707) = +70.7%  ← 左后推
M4: 0 + 0 + 100*(-0.707) = -70.7%  ← 右后拉
```
**修正**: 左侧(M1+M3)推，右侧(M2+M4)拉 → 纯横移右！

---

### Test 2: Forward + Lateral (Diagonal)
**命令**: `throttle=+100, steering=0, lateral=+100`

**修改前输出**:
```
M1: 100*0.707 + 0 + 100*(-0.707) = +70.7 - 70.7 = 0%
M2: 100*0.707 + 0 + 100*(+0.707) = +70.7 + 70.7 = +141.4% → 归一化到 +100%
M3: 100*(-0.707) + 0 + 100*(+0.707) = -70.7 + 70.7 = 0%
M4: 100*(-0.707) + 0 + 100*(-0.707) = -70.7 - 70.7 = -141.4% → 归一化到 -100%
```
**问题**: M2和M4饱和，M1和M3为0 → 沿M2-M4轴运动（非45°）

**修改后输出**:
```
M1: 100*0.707 + 0 + 100*(+0.707) = +70.7 + 70.7 = +141.4% → 归一化到 +100%
M2: 100*0.707 + 0 + 100*(-0.707) = +70.7 - 70.7 = 0%
M3: 100*(-0.707) + 0 + 100*(+0.707) = -70.7 + 70.7 = 0%
M4: 100*(-0.707) + 0 + 100*(-0.707) = -70.7 - 70.7 = -141.4% → 归一化到 -100%
```
**修正**: M1推，M4拉 → 沿右前45°方向运动（正确！）

---

### Test 3: Yaw Rotation
**命令**: `throttle=0, steering=+100, lateral=0`

**修改前输出**:
```
M1: 0 + 100*(-0.707) + 0 = -70.7%
M2: 0 + 100*(+0.707) + 0 = +70.7%
M3: 0 + 100*(-0.707) + 0 = -70.7%
M4: 0 + 100*(+0.707) + 0 = +70.7%
```

**修改后输出**:
```
M1: 0 + 100*(-0.707) + 0 = -70.7%
M2: 0 + 100*(+0.707) + 0 = +70.7%
M3: 0 + 100*(-0.707) + 0 = -70.7%
M4: 0 + 100*(+0.707) + 0 = +70.7%
```

**结论**: Yaw控制不变（因为steering因子未修改）

---

## ArduSub VECTORED 对比验证

### ArduSub M3 (左前) 因子
```cpp
// ArduSub VECTORED M3
add_motor_raw_6dof(MOT_3, 0, 0, -1.0f, 0, 1.0f, 1.0f, 3);
//                         ↑   ↑   ↑     ↑    ↑      ↑
//                        Roll Pitch Yaw Thr Fwd   Lat
```

**输出计算**:
```cpp
linear_out[M3] = throttle_thrust * 0 +
                 forward_thrust * 1.0f +
                 lateral_thrust * 1.0f;

rpy_out[M3] = yaw_thrust * -1.0f;

final_out[M3] = linear_out[M3] + rpy_out[M3];
```

**对应 Rover M1** (修改后):
```cpp
add_omni_motor(0, cos45, -cos45, cos45);
//                 ↑       ↑       ↑
//              Throttle Steering Lateral
//              (=Forward) (=Yaw)  (=Lateral)
```

**输出计算**:
```cpp
thr_str_ltr_out[M1] = throttle * 0.707 +      // = ArduSub forward * 0.707
                      steering * -0.707 +     // = ArduSub yaw * -0.707
                      lateral * 0.707;        // = ArduSub lateral * 0.707
```

**对比验证**:
- ArduSub: Forward=+1, Yaw=-1, Lateral=+1
- Rover: Throttle=+0.707, Steering=-0.707, Lateral=+0.707
- **结论**: Rover = ArduSub * 0.707 → 一致！

---

## 物理意义解释

### 为什么 cos(45°) = 0.707？

X型推进器安装在45°角，推力需要投影到X轴和Y轴：

```
      ↑ Y (Lateral)
      │
      │    M2 ↖ (45°)
      │   /
      │  /
      │ /
      │/____________→ X (Forward)
     O

推力分解：
- X方向分量 = Thrust * cos(45°) = Thrust * 0.707
- Y方向分量 = Thrust * sin(45°) = Thrust * 0.707
```

所以每个推进器对Forward和Lateral的贡献都是`0.707 * 推力`。

### 为什么左侧Lateral为正？

**物理逻辑**：
- 左侧电机(M1, M3)推力向右 → lateral = +1 → 船体向右移动
- 右侧电机(M2, M4)推力向左 → lateral = -1 → 船体向右移动

**修正前的错误**：
- M1推，M2拉，M3拉，M4推 → 形成对角线力矩 → 横移+旋转

**修正后的正确**：
- M1+M3推，M2+M4拉 → 左右对称 → 纯横移

---

## 实施步骤

### 步骤1: 备份原文件
```bash
cd F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf
cp libraries/AR_Motors/AP_MotorsUGV.cpp libraries/AR_Motors/AP_MotorsUGV.cpp.backup
```

### 步骤2: 编辑文件
打开 `libraries/AR_Motors/AP_MotorsUGV.cpp`，定位到行727和730：

**修改行727** (M1的lateral因子):
```cpp
// 修改前
add_omni_motor(0, cos45, -cos45, -cos45);

// 修改后
add_omni_motor(0, cos45, -cos45, cos45);
```

**修改行730** (M2的lateral因子):
```cpp
// 修改前
add_omni_motor(1, cos45, cos45, cos45);

// 修改后
add_omni_motor(1, cos45, cos45, -cos45);
```

### 步骤3: 编译
```bash
cd F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf
D:\Espressif\v5.5.1\esp-idf\tools\idf.py build
```

### 步骤4: 刷写
```bash
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM3 flash
```

### 步骤5: 测试
1. 连接地面站
2. 设置 `FRAME_TYPE=11`
3. 重启ESP32
4. 执行Lateral测试（油门=0，转向=0，横移=100）
5. 观察船体是否纯横移（无旋转）

---

## 如果需要回滚

### 方法1: 恢复备份
```bash
cp libraries/AR_Motors/AP_MotorsUGV.cpp.backup libraries/AR_Motors/AP_MotorsUGV.cpp
```

### 方法2: 手动修改
将M1和M2的lateral因子改回原值：
```cpp
add_omni_motor(0, cos45, -cos45, -cos45);  // M1
add_omni_motor(1, cos45, cos45, cos45);    // M2
```

---

## FAQ

### Q1: 为什么只修改M1和M2，不修改M3和M4？
**A**: M3和M4的lateral因子本来就是正确的（+0.707和-0.707），无需修改。

### Q2: 修改后会影响Forward和Yaw吗？
**A**: 不会。只修改了lateral因子，throttle和steering因子保持不变。

### Q3: 如何验证修改是否正确？
**A**: 执行Pure Lateral测试，观察船体是否纯横移（无旋转）。如果有旋转，说明因子不对称。

### Q4: 这个修改适用于其他配置吗？
**A**: 不适用。仅适用于 `FRAME_TYPE_BOAT_VECTORED_X` (11)。

### Q5: 修改后需要重新调参吗？
**A**: 可能需要。Lateral PID参数可能需要微调，因为运动特性改变了。

---

## 技术细节

### output_omni() 函数逻辑
```cpp
// AP_MotorsUGV.cpp 行999-1055
void AP_MotorsUGV::output_omni(bool armed, float steering, float throttle, float lateral)
{
    if (armed) {
        // 归一化输入
        const float scaled_throttle = throttle * 0.01f;      // -1 to +1
        const float scaled_steering = steering / 4500.0f;    // -1 to +1
        const float scaled_lateral = lateral * 0.01f;        // -1 to +1

        // 混合计算
        float thr_str_ltr_out[_motors_num];
        float thr_str_ltr_max = 1;
        for (uint8_t i=0; i<_motors_num; i++) {
            thr_str_ltr_out[i] = (scaled_throttle * _throttle_factor[i]) +
                                 (scaled_steering * _steering_factor[i]) +
                                 (scaled_lateral * _lateral_factor[i]);
            if (fabsf(thr_str_ltr_out[i]) > thr_str_ltr_max) {
                thr_str_ltr_max = fabsf(thr_str_ltr_out[i]);
            }
        }

        // 归一化输出（防止饱和）
        const float output_scale = 1 / thr_str_ltr_max;
        for (uint8_t i=0; i<_motors_num; i++) {
            output_throttle(
                SRV_Channels::get_motor_function(i),
                thr_str_ltr_out[i] * 100.0f * output_scale
            );
        }
    }
}
```

**关键点**：
1. 输入归一化到 [-1, +1]
2. 线性混合所有因子
3. 自动归一化防止饱和（保持方向）

---

## 参考文档

1. **ArduSub VECTORED配置**: `F:\opensource\usv_esp32\ardupilot-master\libraries\AP_Motors\AP_Motors6DOF.cpp` 行168-176
2. **Rover omni输出**: `F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\libraries\AR_Motors\AP_MotorsUGV.cpp` 行999-1055
3. **深度分析文档**: `docs/ARDUSUB_CONTROL_ANALYSIS.md`

---

**文档版本**: v1.0
**作者**: ArduPilot ESP32-S3 Team
**日期**: 2025-11-09
**状态**: 待测试
