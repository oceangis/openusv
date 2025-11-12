# ArduSub 6DOF vs Rover 3因子控制深度分析

## 文档信息
- **创建日期**: 2025-11-09
- **分析对象**: ESP32-S3 USV X型船（FRAME_TYPE_BOAT_VECTORED_X）
- **目标**: 使用 ArduSub 风格的控制改进当前 Rover 的3因子控制

---

## 1. 控制架构深度对比

### 1.1 ArduSub 6DOF 控制架构

**源文件**: `F:\opensource\usv_esp32\ardupilot-master\libraries\AP_Motors\AP_Motors6DOF.cpp`

#### 控制自由度（6个）
```cpp
roll_thrust     // Roll  姿态控制 [-1, +1]
pitch_thrust    // Pitch 姿态控制 [-1, +1]
yaw_thrust      // Yaw   航向控制 [-1, +1]
throttle_thrust // 垂直推力控制  [-1, +1]
forward_thrust  // 前后推力控制  [-1, +1]
lateral_thrust  // 左右推力控制  [-1, +1]
```

#### 输出计算方式（行327-350）
```cpp
// 第一步：计算 RPY 分量（姿态控制）
for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
    if (motor_enabled[i]) {
        rpy_out[i] = roll_thrust * _roll_factor[i] +
                     pitch_thrust * _pitch_factor[i] +
                     yaw_thrust * _yaw_factor[i];
    }
}

// 第二步：计算 Linear 分量（线性运动）
for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
    if (motor_enabled[i]) {
        linear_out[i] = throttle_thrust * _throttle_factor[i] +
                        forward_thrust * _forward_factor[i] +
                        lateral_thrust * _lateral_factor[i];
    }
}

// 第三步：合成最终输出
for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
    if (motor_enabled[i]) {
        _thrust_rpyt_out[i] = constrain_float(
            _motor_reverse[i] * (rpy_out[i] + linear_out[i]),
            -1.0f, 1.0f
        );
    }
}
```

**关键特点**：
- **分离式架构**：RPY（姿态）和Linear（运动）独立计算后合成
- **6个独立控制轴**：每个轴独立控制，互不干扰
- **适用场景**：ROV/AUV 水下6自由度全姿态控制

---

### 1.2 Rover 3因子控制架构

**源文件**: `F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\libraries\AR_Motors\AP_MotorsUGV.cpp`

#### 控制自由度（3个）
```cpp
throttle  // 前后速度控制 [-100, +100]
steering  // 转向/偏航控制 [-4500, +4500]
lateral   // 左右平移控制 [-100, +100]
```

#### 输出计算方式（行1020-1034）
```cpp
// 单步计算：直接混合所有因子
for (uint8_t i=0; i<_motors_num; i++) {
    thr_str_ltr_out[i] = (scaled_throttle * _throttle_factor[i]) +
                         (scaled_steering * _steering_factor[i]) +
                         (scaled_lateral * _lateral_factor[i]);
}

// 归一化处理（防止饱和）
const float output_scale = 1 / thr_str_ltr_max;
for (uint8_t i=0; i<_motors_num; i++) {
    output_throttle(
        SRV_Channels::get_motor_function(i),
        thr_str_ltr_out[i] * 100.0f * output_scale
    );
}
```

**关键特点**：
- **融合式架构**：所有控制直接线性混合
- **3个融合控制轴**：throttle包含forward，steering包含yaw
- **适用场景**：USV 水面2D平面+偏航控制

---

## 2. 布局和因子深度对比

### 2.1 ArduSub VECTORED 配置（前4个水平电机）

**源文件**: `AP_Motors6DOF.cpp` 行168-176

```cpp
//                   Motor #  Roll  Pitch  Yaw     Throttle  Forward  Lateral
add_motor_raw_6dof(MOT_1,     0,    0,     1.0f,   0,        -1.0f,   1.0f,    1);
add_motor_raw_6dof(MOT_2,     0,    0,    -1.0f,   0,        -1.0f,  -1.0f,    2);
add_motor_raw_6dof(MOT_3,     0,    0,    -1.0f,   0,         1.0f,   1.0f,    3);
add_motor_raw_6dof(MOT_4,     0,    0,     1.0f,   0,         1.0f,  -1.0f,    4);
```

#### 物理布局
```
      Bow ↑

   M3 ↗  ↖ M4   (前)
      \  /
       \/
       /\
      /  \
   M1 ↙  ↘ M2   (后)

电机位置说明：
M1: 左后 (Port-Aft)     - Yaw=+1, Forward=-1, Lateral=+1
M2: 右后 (Starboard-Aft)- Yaw=-1, Forward=-1, Lateral=-1
M3: 左前 (Port-Fwd)     - Yaw=-1, Forward=+1, Lateral=+1
M4: 右前 (Starboard-Fwd)- Yaw=+1, Forward=+1, Lateral=-1
```

#### 因子分析表

| Motor | Roll | Pitch | Yaw  | Throttle | Forward | Lateral | 物理位置 |
|-------|------|-------|------|----------|---------|---------|----------|
| M1    | 0    | 0     | +1   | 0        | -1      | +1      | 左后     |
| M2    | 0    | 0     | -1   | 0        | -1      | -1      | 右后     |
| M3    | 0    | 0     | -1   | 0        | +1      | +1      | 左前     |
| M4    | 0    | 0     | +1   | 0        | +1      | -1      | 右前     |

**设计逻辑**：
- **Forward运动**：前2个电机(M3,M4)正转，后2个(M1,M2)正转，推力方向相同
- **Yaw旋转**：对角线电机(M1,M4)同向，另一对角线(M2,M3)反向
- **Lateral平移**：左侧(M1,M3)同向，右侧(M2,M4)反向

---

### 2.2 Rover X型配置（当前实现）

**源文件**: `AP_MotorsUGV.cpp` 行704-739

```cpp
const float cos45 = 0.70710678f;  // cos(45°)

//                   Motor #  Throttle   Steering   Lateral
add_omni_motor(0,              cos45,     -cos45,    -cos45);   // M1
add_omni_motor(1,              cos45,      cos45,     cos45);   // M2
add_omni_motor(2,             -cos45,     -cos45,     cos45);   // M3
add_omni_motor(3,             -cos45,      cos45,    -cos45);   // M4
```

#### 物理布局
```
      Bow ↑

   M1 ↗  ↖ M2   (前)
      \  /
       \/
       /\
      /  \
   M3 ↙  ↘ M4   (后)

电机位置说明：
M1: 左前45° - Throttle=+0.707, Steering=-0.707, Lateral=-0.707
M2: 右前45° - Throttle=+0.707, Steering=+0.707, Lateral=+0.707
M3: 左后45° - Throttle=-0.707, Steering=-0.707, Lateral=+0.707
M4: 右后45° - Throttle=-0.707, Steering=+0.707, Lateral=-0.707
```

#### 因子分析表

| Motor | Throttle | Steering | Lateral | 物理位置 | 推力方向 |
|-------|----------|----------|---------|----------|----------|
| M1    | +0.707   | -0.707   | -0.707  | 左前     | ↗ 45°    |
| M2    | +0.707   | +0.707   | +0.707  | 右前     | ↖ 45°    |
| M3    | -0.707   | -0.707   | +0.707  | 左后     | ↙ 45°    |
| M4    | -0.707   | +0.707   | -0.707  | 右后     | ↘ 45°    |

**设计逻辑**：
- **Throttle前进**：前2个(M1,M2)正推力，后2个(M3,M4)负推力
- **Steering右转**：右侧(M2,M4)正因子，左侧(M1,M3)负因子
- **Lateral右移**：右侧(M2,M4)正因子，左侧(M1,M3)负因子

---

## 3. 关键差异总结

### 3.1 架构差异

| 维度 | ArduSub 6DOF | Rover 3因子 |
|------|-------------|------------|
| **控制自由度** | 6 (roll, pitch, yaw, throttle, forward, lateral) | 3 (throttle, steering, lateral) |
| **计算架构** | 分离式（RPY + Linear） | 融合式（单次混合） |
| **Roll/Pitch** | 支持（水下姿态控制） | 不支持（水面不需要） |
| **Yaw控制** | 独立的 yaw_thrust | 融合在 steering 中 |
| **Forward控制** | 独立的 forward_thrust | 融合在 throttle 中 |
| **适用场景** | ROV/AUV 6DOF水下 | USV 3DOF水面 |

### 3.2 因子映射关系

| ArduSub 6DOF | Rover 3因子 | 说明 |
|--------------|------------|------|
| `forward_thrust` | `throttle` | 前后运动（语义不同） |
| `yaw_thrust` | `steering` | 偏航控制（单位不同） |
| `lateral_thrust` | `lateral` | 横移控制（单位不同） |
| `throttle_thrust` | - | 垂直控制（USV不需要） |
| `roll_thrust` | - | 翻滚控制（USV不需要） |
| `pitch_thrust` | - | 俯仰控制（USV不需要） |

### 3.3 布局差异（X型配置）

**ArduSub VECTORED（假设电机布局）**：
```
对比项               M1(左后)  M2(右后)  M3(左前)  M4(右前)
Yaw因子             +1        -1        -1        +1
Forward因子         -1        -1        +1        +1
Lateral因子         +1        -1        +1        -1
```

**Rover X型（当前）**：
```
对比项               M1(左前)  M2(右前)  M3(左后)  M4(右后)
Throttle因子        +0.707    +0.707    -0.707    -0.707
Steering因子        -0.707    +0.707    -0.707    +0.707
Lateral因子         -0.707    +0.707    +0.707    -0.707
```

**关键发现**：
1. **电机编号顺序不同**：ArduSub 后前布局，Rover 前后布局
2. **因子值不同**：ArduSub 使用±1，Rover 使用±0.707（cos45°）
3. **符号体系不同**：需要仔细映射

---

## 4. 优缺点分析

### 4.1 ArduSub 6DOF 优点
1. **解耦设计**：姿态控制和运动控制分离，逻辑清晰
2. **扩展性强**：支持完整6自由度，易于添加新功能
3. **独立调节**：每个轴独立PID调参
4. **水下ROV标准**：经过大量水下测试验证

### 4.2 ArduSub 6DOF 缺点
1. **计算复杂**：两次循环计算，CPU开销大
2. **过度设计**：USV不需要roll/pitch/throttle
3. **参数冗余**：6个输入只用3个
4. **代码移植**：需要大量修改Rover架构

### 4.3 Rover 3因子 优点
1. **计算高效**：单次混合，CPU友好
2. **简洁实用**：仅包含USV需要的3个自由度
3. **架构轻量**：无需复杂的RPY分离
4. **参数精简**：3个输入正好匹配需求

### 4.4 Rover 3因子 缺点
1. **因子不明确**：throttle混合了forward语义
2. **缺少标准**：与ROV社区标准不一致
3. **调试困难**：throttle/steering混合难以独立调参
4. **扩展受限**：难以添加新的控制模式

---

## 5. 设计决策：两种改进方案

### 方案A：保持3因子，优化因子计算（推荐）

**理由**：
- 不破坏Rover架构
- 保持计算效率
- 最小化代码修改
- ESP32-S3资源友好

**改进思路**：
1. **重新设计因子值**，参考ArduSub VECTORED的逻辑
2. **优化符号体系**，使throttle → forward，steering → yaw语义更清晰
3. **添加详细注释**，说明与ArduSub的对应关系

**优点**：
- 改动最小（仅修改因子数值）
- 性能无损失
- 兼容现有代码

**缺点**：
- 仍然是融合式架构
- 无法独立调试forward/yaw

---

### 方案B：扩展为6因子，模拟ArduSub（不推荐）

**理由**：
- 完全复制ArduSub架构
- 分离RPY和Linear计算
- 增加roll/pitch/throttle输入（虽然不用）

**需要修改**：
1. `AP_MotorsUGV.h`：添加6个输入变量
2. `AP_MotorsUGV.cpp`：重写`output_omni()`为两阶段计算
3. `mode.cpp`：修改所有模式的控制输出
4. 大量参数和逻辑调整

**优点**：
- 与ArduSub完全一致
- 理论上更易理解

**缺点**：
- **大量代码修改**（估计500+行）
- **破坏Rover架构**（可能影响其他配置）
- **性能损失**（双循环计算）
- **参数冗余**（3个无用输入）
- **测试复杂**（影响所有模式）

---

## 6. 推荐方案详细设计

### 6.1 方案A：优化因子计算

#### 设计原则
1. **语义映射**：
   - Rover `throttle` → ArduSub `forward`
   - Rover `steering` → ArduSub `yaw`
   - Rover `lateral` → ArduSub `lateral`

2. **因子重新设计**：
   - 参考ArduSub VECTORED的±1因子逻辑
   - 保持cos(45°)的物理意义
   - 统一符号体系

3. **电机编号对齐**：
   - 统一为"前后左右"的逻辑顺序
   - 与ArduSub的物理布局对应

#### 当前因子（需要改进）
```cpp
// Rover当前 X型 (前后左右顺序)
//                Motor  Throttle   Steering   Lateral
add_omni_motor(0,        cos45,     -cos45,    -cos45);   // M1 左前
add_omni_motor(1,        cos45,      cos45,     cos45);   // M2 右前
add_omni_motor(2,       -cos45,     -cos45,     cos45);   // M3 左后
add_omni_motor(3,       -cos45,      cos45,    -cos45);   // M4 右后
```

**问题分析**：
- Lateral因子不对称（M1=-0.707, M2=+0.707）
- 与ArduSub的Lateral符号相反
- 难以与ArduSub VECTORED对应

#### 改进后因子（推荐）
```cpp
// 改进的 X型 (对齐ArduSub VECTORED逻辑)
//                Motor  Throttle   Steering   Lateral    物理意义
add_omni_motor(0,        cos45,     -cos45,     cos45);   // M1 左前 (对应Sub M3)
add_omni_motor(1,        cos45,      cos45,    -cos45);   // M2 右前 (对应Sub M4)
add_omni_motor(2,       -cos45,     -cos45,     cos45);   // M3 左后 (对应Sub M1)
add_omni_motor(3,       -cos45,      cos45,    -cos45);   // M4 右后 (对应Sub M2)
```

**改进说明**：
1. **Lateral符号修正**：M1/M3(左侧)=+0.707, M2/M4(右侧)=-0.707
2. **对应ArduSub**：
   - Rover M1 ≈ ArduSub M3（左前，Forward=+1, Lateral=+1, Yaw=-1）
   - Rover M2 ≈ ArduSub M4（右前，Forward=+1, Lateral=-1, Yaw=+1）
   - Rover M3 ≈ ArduSub M1（左后，Forward=-1, Lateral=+1, Yaw=+1）
   - Rover M4 ≈ ArduSub M2（右后，Forward=-1, Lateral=-1, Yaw=-1）

#### 对比验证表

**Forward运动（throttle=+1, steering=0, lateral=0）**：

| Motor | 当前因子 | 当前输出 | 改进因子 | 改进输出 | ArduSub等效 |
|-------|---------|---------|---------|---------|------------|
| M1    | +0.707  | +0.707  | +0.707  | +0.707  | M3: +1     |
| M2    | +0.707  | +0.707  | +0.707  | +0.707  | M4: +1     |
| M3    | -0.707  | -0.707  | -0.707  | -0.707  | M1: -1     |
| M4    | -0.707  | -0.707  | -0.707  | -0.707  | M2: -1     |

**Yaw右转（throttle=0, steering=+1, lateral=0）**：

| Motor | 当前因子 | 当前输出 | 改进因子 | 改进输出 | ArduSub等效 |
|-------|---------|---------|---------|---------|------------|
| M1    | -0.707  | -0.707  | -0.707  | -0.707  | M3: -1     |
| M2    | +0.707  | +0.707  | +0.707  | +0.707  | M4: +1     |
| M3    | -0.707  | -0.707  | -0.707  | -0.707  | M1: +1     |
| M4    | +0.707  | +0.707  | +0.707  | +0.707  | M2: -1     |

**Lateral右移（throttle=0, steering=0, lateral=+1）**：

| Motor | 当前因子 | 当前输出 | 改进因子 | 改进输出 | ArduSub等效 |
|-------|---------|---------|---------|---------|------------|
| M1    | -0.707  | -0.707  | +0.707  | +0.707  | M3: +1     |
| M2    | +0.707  | +0.707  | -0.707  | -0.707  | M4: -1     |
| M3    | +0.707  | +0.707  | +0.707  | +0.707  | M1: +1     |
| M4    | -0.707  | -0.707  | -0.707  | -0.707  | M2: -1     |

**关键发现**：
- **Forward/Yaw方向不变**：改进前后一致
- **Lateral方向修正**：M1/M2符号对调，更符合物理直觉
- **ArduSub对应清晰**：每个电机可以映射到Sub的对应电机

---

### 6.2 代码修改清单

**文件**: `libraries/AR_Motors/AP_MotorsUGV.cpp`

**位置**: 行704-739 `FRAME_TYPE_BOAT_VECTORED_X` case

**修改前**：
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
    add_omni_motor(0, cos45, -cos45, -cos45);

    // Motor 2: Right-front 45°
    add_omni_motor(1, cos45, cos45, cos45);

    // Motor 3: Left-rear 45°
    add_omni_motor(2, -cos45, -cos45, cos45);

    // Motor 4: Right-rear 45°
    add_omni_motor(3, -cos45, cos45, -cos45);

    gcs().send_text(MAV_SEVERITY_INFO, "Motors: Boat X-config (4 thrusters)");
    break;
```

**修改后**：
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
     * - throttle → forward_thrust  (前后运动)
     * - steering → yaw_thrust      (偏航旋转)
     * - lateral  → lateral_thrust  (横向平移)
     *
     * Motor Correspondence (Rover → ArduSub VECTORED):
     * - M1 (左前) → Sub M3 (Forward=+1, Lateral=+1, Yaw=-1)
     * - M2 (右前) → Sub M4 (Forward=+1, Lateral=-1, Yaw=+1)
     * - M3 (左后) → Sub M1 (Forward=-1, Lateral=+1, Yaw=+1)
     * - M4 (右后) → Sub M2 (Forward=-1, Lateral=-1, Yaw=-1)
     *
     * Physical Behavior:
     * - Forward: M1+M2 push, M3+M4 pull (differential thrust)
     * - Yaw CW: M2+M3 push, M1+M4 pull (diagonal rotation)
     * - Lateral Right: M2+M4 pull, M1+M3 push (sideways translation)
     *
     * Capabilities:
     * - Omnidirectional movement
     * - Rotate in place
     * - Pure lateral translation
     * - Redundant (works with any 3 thrusters)
     */
    _motors_num = 4;

    const float cos45 = 0.70710678f;  // cos(45°) = 1/√2

    // Motor 1: Left-front 45° (Port-Forward)
    // add_omni_motor(motor_num, throttle_factor, steering_factor, lateral_factor)
    // Corresponds to ArduSub VECTORED M3
    add_omni_motor(0, cos45, -cos45, cos45);

    // Motor 2: Right-front 45° (Starboard-Forward)
    // Corresponds to ArduSub VECTORED M4
    add_omni_motor(1, cos45, cos45, -cos45);

    // Motor 3: Left-rear 45° (Port-Aft)
    // Corresponds to ArduSub VECTORED M1
    add_omni_motor(2, -cos45, -cos45, cos45);

    // Motor 4: Right-rear 45° (Starboard-Aft)
    // Corresponds to ArduSub VECTORED M2
    add_omni_motor(3, -cos45, cos45, -cos45);

    gcs().send_text(MAV_SEVERITY_INFO, "Motors: Boat X-config (ArduSub style, 4 thrusters)");
    break;
```

---

## 7. 测试验证计划

### 7.1 单因子测试

#### Test 1: Pure Forward (throttle=+50, steering=0, lateral=0)

**预期输出**：
```
M1 (左前): +50 * 0.707 = +35.4%
M2 (右前): +50 * 0.707 = +35.4%
M3 (左后): +50 * -0.707 = -35.4%
M4 (右后): +50 * -0.707 = -35.4%
```

**预期行为**：船体前进，无横移，无偏航

---

#### Test 2: Pure Yaw CW (throttle=0, steering=+100, lateral=0)

**预期输出**：
```
M1 (左前): +100 * -0.707 = -70.7%
M2 (右前): +100 * 0.707 = +70.7%
M3 (左后): +100 * -0.707 = -70.7%
M4 (右后): +100 * 0.707 = +70.7%
```

**预期行为**：船体顺时针旋转，无平移

---

#### Test 3: Pure Lateral Right (throttle=0, steering=0, lateral=+100)

**改进前输出**：
```
M1 (左前): +100 * -0.707 = -70.7%
M2 (右前): +100 * 0.707 = +70.7%
M3 (左后): +100 * 0.707 = +70.7%
M4 (右后): +100 * -0.707 = -70.7%
```
**问题**：M1推，M2拉，M3拉，M4推 → 横移+旋转！

**改进后输出**：
```
M1 (左前): +100 * 0.707 = +70.7%
M2 (右前): +100 * -0.707 = -70.7%
M3 (左后): +100 * 0.707 = +70.7%
M4 (右后): +100 * -0.707 = -70.7%
```
**修正**：M1+M3推，M2+M4拉 → 纯横移右！

---

### 7.2 组合运动测试

#### Test 4: Forward + Yaw (throttle=+50, steering=+50, lateral=0)

**预期输出**：
```
M1: (+50*0.707) + (+50*-0.707) = +35.4 - 35.4 = 0%
M2: (+50*0.707) + (+50*0.707) = +35.4 + 35.4 = +70.8%
M3: (+50*-0.707) + (+50*-0.707) = -35.4 - 35.4 = -70.8%
M4: (+50*-0.707) + (+50*0.707) = -35.4 + 35.4 = 0%
```

**预期行为**：船体前进同时右转（类似汽车右转）

---

#### Test 5: Diagonal Movement (throttle=+50, steering=0, lateral=+50)

**改进后输出**：
```
M1: (+50*0.707) + (+50*0.707) = +70.8%
M2: (+50*0.707) + (+50*-0.707) = 0%
M3: (+50*-0.707) + (+50*0.707) = 0%
M4: (+50*-0.707) + (+50*-0.707) = -70.8%
```

**预期行为**：船体沿右前45°方向移动（M1推，M4拉）

---

### 7.3 边界条件测试

#### Test 6: Saturation Handling (throttle=+100, steering=+100, lateral=0)

**原始输出**：
```
M1: (+100*0.707) + (+100*-0.707) = 0%
M2: (+100*0.707) + (+100*0.707) = +141.4% → 饱和！
M3: (+100*-0.707) + (+100*-0.707) = -141.4% → 饱和！
M4: (+100*-0.707) + (+100*0.707) = 0%
```

**归一化后输出** (output_scale = 1/1.414):
```
M1: 0%
M2: +100%
M3: -100%
M4: 0%
```

**预期行为**：前进和右转按比例缩放，保持运动方向

---

### 7.4 ArduSub对比测试

#### Test 7: 验证与ArduSub VECTORED的一致性

**测试设置**：
- Rover: throttle=+50, steering=0, lateral=+50
- ArduSub: forward=+0.5, yaw=0, lateral=+0.5

**Rover输出**（改进后）：
```
M1: +70.7%
M2: 0%
M3: 0%
M4: -70.7%
```

**ArduSub VECTORED输出**（预期）：
```
M3 (对应Rover M1): (+0.5*1) + (+0.5*1) = +1.0 → +100%
M4 (对应Rover M2): (+0.5*1) + (+0.5*-1) = 0 → 0%
M1 (对应Rover M3): (+0.5*-1) + (+0.5*1) = 0 → 0%
M2 (对应Rover M4): (+0.5*-1) + (+0.5*-1) = -1.0 → -100%
```

**归一化后**（因为cos45=0.707）：
```
ArduSub归一化: 100% * 0.707 = 70.7%
```

**结论**：Rover改进后输出 = ArduSub * 0.707 → 一致！

---

## 8. 实施步骤

### 第一阶段：代码修改（10分钟）
1. 备份 `AP_MotorsUGV.cpp`
2. 修改 `FRAME_TYPE_BOAT_VECTORED_X` case
3. 更新注释和文档

### 第二阶段：编译测试（5分钟）
1. 执行 `idf.py build`
2. 检查编译错误
3. 验证二进制大小

### 第三阶段：基础功能测试（30分钟）
1. 上传固件到ESP32-S3
2. 设置 `FRAME_TYPE=11`
3. 执行单因子测试（Test 1-3）
4. 记录实际输出

### 第四阶段：综合测试（1小时）
1. 组合运动测试（Test 4-5）
2. 边界条件测试（Test 6）
3. 实船测试（如果有条件）
4. 与ArduSub对比（Test 7）

### 第五阶段：文档和总结（30分钟）
1. 记录测试结果
2. 更新参数文档
3. 提交代码和文档

---

## 9. 风险评估

### 9.1 低风险项
- **代码改动小**：仅修改4行因子数值
- **不影响其他配置**：仅FRAME_TYPE=11受影响
- **易于回滚**：保留原始代码注释

### 9.2 中等风险项
- **Lateral方向变化**：M1/M2符号对调，需要重新校准
- **用户习惯**：已经使用旧因子的用户需要适应

### 9.3 缓解措施
1. 详细文档说明变化
2. 提供迁移指南
3. 保留旧因子作为注释
4. 增加版本号标识

---

## 10. 结论

### 10.1 推荐方案
**采用方案A：优化因子计算**
- 最小化修改（仅4行因子数值）
- 保持Rover架构完整性
- 提升与ArduSub的一致性
- ESP32-S3资源友好

### 10.2 核心改进
1. **Lateral因子修正**：M1/M3=+0.707, M2/M4=-0.707
2. **ArduSub映射清晰**：每个电机对应Sub VECTORED电机
3. **物理行为直觉**：左侧推=右移，符合直觉

### 10.3 不推荐方案B的原因
- 需要大量代码修改（500+行）
- 破坏Rover架构（影响稳定性）
- 性能损失（双循环计算）
- 参数冗余（3个无用输入）
- 测试复杂度指数增长

### 10.4 最终建议
**立即实施方案A**，通过最小化改动获得最大收益。

---

## 附录A：完整因子对比表

| Motor | 位置 | 当前Throttle | 当前Steering | 当前Lateral | 改进Throttle | 改进Steering | 改进Lateral | ArduSub对应 |
|-------|------|-------------|-------------|------------|-------------|-------------|------------|------------|
| M1    | 左前 | +0.707      | -0.707      | **-0.707** | +0.707      | -0.707      | **+0.707** | M3         |
| M2    | 右前 | +0.707      | +0.707      | **+0.707** | +0.707      | +0.707      | **-0.707** | M4         |
| M3    | 左后 | -0.707      | -0.707      | **+0.707** | -0.707      | -0.707      | **+0.707** | M1         |
| M4    | 右后 | -0.707      | +0.707      | **-0.707** | -0.707      | +0.707      | **-0.707** | M2         |

**加粗部分**：修改的因子

---

## 附录B：ArduSub VECTORED完整配置

```cpp
// ArduSub VECTORED (6 thrusters, only front 4 are horizontal)
//                   Motor #  Roll  Pitch  Yaw     Throttle  Forward  Lateral
add_motor_raw_6dof(MOT_1,     0,    0,     1.0f,   0,        -1.0f,   1.0f,    1);  // 左后
add_motor_raw_6dof(MOT_2,     0,    0,    -1.0f,   0,        -1.0f,  -1.0f,    2);  // 右后
add_motor_raw_6dof(MOT_3,     0,    0,    -1.0f,   0,         1.0f,   1.0f,    3);  // 左前
add_motor_raw_6dof(MOT_4,     0,    0,     1.0f,   0,         1.0f,  -1.0f,    4);  // 右前
add_motor_raw_6dof(MOT_5,     1.0f, 0,     0,      -1.0f,     0,      0,       5);  // 垂直
add_motor_raw_6dof(MOT_6,    -1.0f, 0,     0,      -1.0f,     0,      0,       6);  // 垂直
```

**输出计算**：
```cpp
// RPY分量
rpy_out[i] = roll_thrust * _roll_factor[i] +
             pitch_thrust * _pitch_factor[i] +
             yaw_thrust * _yaw_factor[i];

// Linear分量
linear_out[i] = throttle_thrust * _throttle_factor[i] +
                forward_thrust * _forward_factor[i] +
                lateral_thrust * _lateral_factor[i];

// 最终输出
_thrust_rpyt_out[i] = constrain_float(
    _motor_reverse[i] * (rpy_out[i] + linear_out[i]),
    -1.0f, 1.0f
);
```

---

**文档版本**: v1.0
**作者**: ArduPilot ESP32-S3 Team
**日期**: 2025-11-09
**审核**: 待审核
