# 船用矢量推进配置实现文档

## 修改日期
2025-11-09

## 概述
为 ESP32-S3 USV 添加两种新的船用矢量推进配置：
- **FRAME_TYPE_BOAT_VECTORED_T (10)** - T型：双推+前横向推进器
- **FRAME_TYPE_BOAT_VECTORED_X (11)** - X型：4个45度对称推进器

---

## 文件修改清单

### 1. libraries/AR_Motors/AP_MotorsUGV.h

#### 修改位置：第 24-31 行

**原代码**：
```cpp
    // supported omni motor configurations
    enum frame_type {
        FRAME_TYPE_UNDEFINED = 0,
        FRAME_TYPE_OMNI3 = 1,
        FRAME_TYPE_OMNIX = 2,
        FRAME_TYPE_OMNIPLUS = 3,
        FRAME_TYPE_OMNI3MECANUM = 4,
    };
```

**修改为**：
```cpp
    // Motor frame types for different vehicle configurations
    enum frame_type {
        // Universal - works for all vehicle types
        FRAME_TYPE_UNDEFINED = 0,

        // Ground vehicle omni-directional configurations (1-9)
        // These use omniwheels or mecanum wheels for holonomic movement
        FRAME_TYPE_OMNI3 = 1,              // Three omni wheels in triangle
        FRAME_TYPE_OMNIX = 2,              // Four wheels in X pattern
        FRAME_TYPE_OMNIPLUS = 3,           // Four wheels in + pattern
        FRAME_TYPE_OMNI3MECANUM = 4,       // Three mecanum wheels
        // Reserved 5-9 for future ground vehicle configurations

        // Boat vectored thrust configurations (10+)
        // These use multiple thrusters for boat maneuvering
        FRAME_TYPE_BOAT_VECTORED_T = 10,   // T-configuration: dual rear + front lateral
        FRAME_TYPE_BOAT_VECTORED_X = 11,   // X-configuration: four 45° thrusters
        // Reserved 12+ for future boat configurations
    };
```

#### 新增方法（在第 116 行 `is_omni()` 后面添加）：

```cpp
    // returns true if the vehicle uses boat vectored thrust
    bool is_boat_vectored() const {
        return _frame_type == FRAME_TYPE_BOAT_VECTORED_T ||
               _frame_type == FRAME_TYPE_BOAT_VECTORED_X;
    }
```

---

### 2. libraries/AR_Motors/AP_MotorsUGV.cpp

#### 修改位置：在 `setup_omni()` 函数的 switch 语句中添加（约第 200+ 行）

在现有的 `case FRAME_TYPE_OMNI3MECANUM:` 之后添加：

```cpp
    // === Boat vectored thrust configurations (10+) ===

    case FRAME_TYPE_BOAT_VECTORED_T:
        /*
         * T-Configuration for boats
         * Layout:
         *       Bow ↑
         *
         *      [M1] ← Front lateral thruster (port/starboard)
         *        ┃
         *    [M2]┃[M3] ← Dual rear thrusters (forward/turn)
         *
         * Capabilities:
         * - Forward/reverse: M2+M3 same direction
         * - Rotate in place: M2+M3 opposite
         * - Lateral translation: M1 alone
         * - Excellent docking control
         */
        _motors_num = 3;

        // Motor 1: Front lateral thruster (lateral only)
        // add_omni_motor(motor_num, throttle_factor, steering_factor, lateral_factor)
        add_omni_motor(0, 0.0, 0.0, 1.0);

        // Motor 2: Left rear thruster (throttle + steering)
        add_omni_motor(1, 0.5, -0.5, 0.0);

        // Motor 3: Right rear thruster (throttle + steering)
        add_omni_motor(2, 0.5, 0.5, 0.0);

        gcs().send_text(MAV_SEVERITY_INFO, "Motors: Boat T-config (3 thrusters)");
        break;

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

---

### 3. Rover/Parameters.cpp

#### 修改位置：查找 "FRAME_TYPE" 参数定义（约第 464-470 行）

**原代码**：
```cpp
// @Param: FRAME_TYPE
// @DisplayName: Frame Type
// @Description: Frame Type
// @Values: 0:Default,1:Omni3,2:OmniX,3:OmniPlus,4:Omni3Mecanum
// @User: Standard
// @RebootRequired: True
AP_GROUPINFO("FRAME_TYPE", 24, ParametersG2, frame_type, 0),
```

**修改为**：
```cpp
// @Param: FRAME_TYPE
// @DisplayName: Frame Type
// @Description: Motor configuration type. 0=default (skid/ackermann), 1-4=ground omni configs, 10+=boat vectored configs
// @Values: 0:Default,1:Omni3,2:OmniX,3:OmniPlus,4:Omni3Mecanum,10:BoatVectoredT,11:BoatVectoredX
// @User: Standard
// @RebootRequired: True
AP_GROUPINFO("FRAME_TYPE", 24, ParametersG2, frame_type, 0),
```

---

## 配置使用示例

### T型配置（双推+横向）

```ini
# 基本设置
FRAME_CLASS = 2              # 水面船只
FRAME_TYPE = 10              # BoatVectoredT

# 通道映射
# SRV1 (GPIO xx) = Motor1 - Front lateral thruster
# SRV2 (GPIO xx) = Motor2 - Left rear thruster
# SRV3 (GPIO xx) = Motor3 - Right rear thruster

# 电机方向校准
MOT1_DIRECTION = 1
MOT2_DIRECTION = 1
MOT3_DIRECTION = 1

# 控制参数
CRUISE_SPEED = 2.0
CRUISE_THROTTLE = 50
LOIT_LATERAL = 50
```

### X型配置（4个45度推进器）

```ini
# 基本设置
FRAME_CLASS = 2              # 水面船只
FRAME_TYPE = 11              # BoatVectoredX

# 通道映射
# SRV1 (GPIO xx) = Motor1 - Left-front 45°
# SRV2 (GPIO xx) = Motor2 - Right-front 45°
# SRV3 (GPIO xx) = Motor3 - Left-rear 45°
# SRV4 (GPIO xx) = Motor4 - Right-rear 45°

# 电机方向校准
MOT1_DIRECTION = 1
MOT2_DIRECTION = 1
MOT3_DIRECTION = 1
MOT4_DIRECTION = 1

# 控制参数
CRUISE_SPEED = 2.5
CRUISE_THROTTLE = 50
TURN_RADIUS = 0.5
```

---

## 测试步骤

### 1. 单电机测试
```
1. 设置 FRAME_TYPE = 10 (T型) 或 11 (X型)
2. 重启 ESP32
3. 进入 Manual 模式
4. 分别测试每个电机响应
```

### 2. 组合运动测试
```
T型：
- 油门 → M2+M3 前进/后退
- 转向 → M2+M3 差速旋转
- 横移 → M1 左右移动

X型：
- 油门 → 全向前进/后退
- 转向 → 对角线协同旋转
- 横移 → 左右协同移动
```

### 3. 自动导航测试
```
1. Auto 模式沿航点
2. Loiter 模式保持位置
3. RTL 返航
```

---

## 实现完成时间
2025-11-09

## 作者
ArduPilot ESP32-S3 Team

