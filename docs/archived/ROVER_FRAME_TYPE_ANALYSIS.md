# Rover 船体框架类型和帆船类型分析

## 📋 概述

ArduPilot Rover 支持多种车辆和船体类型，包括地面车辆、水面船只、平衡机器人等。本文档详细分析代码中定义的所有框架类型和帆船配置。

**分析时间**: 2025-10-27
**代码库**: ardupilot_rover_esp32s3_idf
**目标平台**: ESP32-S3 (USV - 无人水面艇)

---

## 🎯 框架类型层次结构

```
Rover 框架系统
├── FRAME_CLASS (顶层分类)
│   ├── FRAME_UNDEFINED (0) - 未定义
│   ├── FRAME_ROVER (1) - 地面车辆 ⚙️
│   ├── FRAME_BOAT (2) - 水面船只 🚢
│   └── FRAME_BALANCEBOT (3) - 平衡机器人 🤖
│
└── FRAME_TYPE (电机配置类型)
    ├── FRAME_TYPE_UNDEFINED (0) - 默认/标准
    ├── FRAME_TYPE_OMNI3 (1) - 三轮全向
    ├── FRAME_TYPE_OMNIX (2) - X型全向
    ├── FRAME_TYPE_OMNIPLUS (3) - +型全向
    └── FRAME_TYPE_OMNI3MECANUM (4) - 三轮麦克纳姆
```

---

## 1️⃣ FRAME_CLASS - 顶层框架分类

### 1.1 定义位置

**文件**: `Rover/defines.h` (lines 90-96)

```cpp
// frame class enum used for FRAME_CLASS parameter
enum frame_class {
    FRAME_UNDEFINED = 0,
    FRAME_ROVER = 1,
    FRAME_BOAT = 2,
    FRAME_BALANCEBOT = 3,
};
```

### 1.2 参数定义

**文件**: `Rover/Parameters.cpp` (lines 419-424)

```cpp
// @Param: FRAME_CLASS
// @DisplayName: Frame Class
// @Description: Frame Class
// @Values: 0:Undefined,1:Rover,2:Boat,3:BalanceBot
// @User: Standard
AP_GROUPINFO("FRAME_CLASS", 16, ParametersG2, frame_class, 1),
```

**参数名称**: `FRAME_CLASS`
**默认值**: `1` (FRAME_ROVER - 地面车辆)
**存储类型**: `AP_Int8`

### 1.3 各类型详解

#### 🔹 FRAME_UNDEFINED (0)
- **用途**: 未定义/未初始化状态
- **适用场景**: 系统初始化前或配置错误时
- **行为**: 使用默认配置

#### 🔹 FRAME_ROVER (1) - 地面车辆
- **用途**: 陆地机器人、汽车、坦克等地面车辆
- **MAVLink类型**: `MAV_TYPE_GROUND_ROVER`
- **特性**:
  - 支持差速转向 (skid steering)
  - 支持阿克曼转向 (Ackermann steering)
  - 可以原地转向
  - 支持全向轮配置
- **典型应用**:
  - 农业机器人
  - 巡检车
  - 运输小车
  - RC越野车

#### 🔹 FRAME_BOAT (2) - 水面船只 🚢
- **用途**: 水面无人艇、帆船、摩托艇等水上载具
- **MAVLink类型**: `MAV_TYPE_SURFACE_BOAT`
- **特性**:
  - 无法原地转向（需要前进速度）
  - 支持矢量推进 (vectored thrust)
  - 支持帆船配置
  - 漂移补偿
  - 水流影响处理
- **检测函数** (`system.cpp:321-324`):
  ```cpp
  bool Rover::is_boat() const
  {
      return ((enum frame_class)g2.frame_class.get() == FRAME_BOAT);
  }
  ```
- **典型应用**:
  - **USV (无人水面艇)** ← 本项目 ✅
  - 自动巡逻船
  - 海洋监测平台
  - 帆船

#### 🔹 FRAME_BALANCEBOT (3) - 平衡机器人 🤖
- **用途**: 两轮自平衡机器人
- **特性**:
  - 使用俯仰角控制油门
  - 需要姿态控制保持平衡
  - 支持前进/后退平衡
- **检测函数** (`balance_bot.cpp:17-20`):
  ```cpp
  bool Rover::is_balancebot() const
  {
      return ((enum frame_class)g2.frame_class.get() == FRAME_BALANCEBOT);
  }
  ```
- **典型应用**:
  - Segway风格机器人
  - 两轮自平衡车
  - 教育机器人

### 1.4 MAVLink 类型映射

**文件**: `Rover/GCS_MAVLink_Rover.cpp` (lines 10-16)

```cpp
MAV_TYPE GCS_Rover::frame_type() const
{
    if (rover.is_boat()) {
        return MAV_TYPE_SURFACE_BOAT;  // 水面船只
    }
    return MAV_TYPE_GROUND_ROVER;  // 地面车辆/平衡机器人
}
```

**映射关系**:
```
FRAME_BOAT (2) → MAV_TYPE_SURFACE_BOAT (11)
其他所有类型 → MAV_TYPE_GROUND_ROVER (10)
```

### 1.5 使用示例

**参数设置** (GCS/Mission Planner):
```
FRAME_CLASS = 2  # 设置为船只模式（USV）
```

**代码检查**:
```cpp
if (rover.is_boat()) {
    // 船只特有的逻辑
    // - 不能原地转向
    // - 需要速度才能转向
    // - 考虑水流影响
}
```

---

## 2️⃣ FRAME_TYPE - 电机配置类型

### 2.1 定义位置

**文件**: `libraries/AR_Motors/AP_MotorsUGV.h` (lines 25-31)

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

### 2.2 参数定义

**文件**: `Rover/Parameters.cpp` (lines 464-470)

```cpp
// @Param: FRAME_TYPE
// @DisplayName: Frame Type
// @Description: Frame Type
// @Values: 0:Default,1:Omni3,2:OmniX,3:OmniPlus,4:Omni3Mecanum
// @User: Standard
// @RebootRequired: True
AP_GROUPINFO("FRAME_TYPE", 24, ParametersG2, frame_type, 0),
```

**参数名称**: `FRAME_TYPE`
**默认值**: `0` (FRAME_TYPE_UNDEFINED - 标准差速/阿克曼转向)
**存储类型**: `AP_Int8`
**需要重启**: ✅ Yes

### 2.3 各类型详解

#### 🔹 FRAME_TYPE_UNDEFINED (0) - 默认/标准

**用途**: 传统转向系统（非全向）

**支持的配置**:
1. **差速转向 (Skid Steering)**
   ```
   [左电机] [右电机]
      ↕         ↕
   通过左右轮速差实现转向
   ```

2. **阿克曼转向 (Ackermann Steering)**
   ```
   [转向舵机]
        ↕
   [驱动电机]
   像汽车一样转向
   ```

3. **矢量推进 (Vectored Thrust)** - 船用
   ```
   [推进器]
        ↕ (可旋转)
   [转向舵机控制推进方向]
   ```

**典型应用**:
- 汽车型机器人
- 坦克型机器人
- **单螺旋桨船只** ← USV 常用 ✅
- RC车

---

#### 🔹 FRAME_TYPE_OMNI3 (1) - 三轮全向

**布局**:
```
         前方
          ↑
      M1 (0°)
      /    \
    M2     M3
  (120°) (240°)

三个电机呈等边三角形分布
每个电机可独立控制速度和方向
```

**运动能力**:
- ✅ 前进/后退
- ✅ 左右平移
- ✅ 原地旋转
- ✅ 任意方向斜向移动

**控制因子**:
```cpp
// 电机输出 = throttle_factor * 油门 + steering_factor * 转向 + lateral_factor * 横移
Motor1: throttle=1.0,  steering=0.0,   lateral=0.0    // 正前方
Motor2: throttle=-0.5, steering=-0.866, lateral=-0.866 // 左后方
Motor3: throttle=-0.5, steering=0.866,  lateral=0.866  // 右后方
```

**应用场景**:
- 室内机器人
- 仓储AGV
- 服务机器人

---

#### 🔹 FRAME_TYPE_OMNIX (2) - X型全向

**布局**:
```
    前方
     ↑
  M1   M2
    \ /
     X
    / \
  M3   M4

四个电机呈X型分布（45°倾角）
```

**运动能力**:
- ✅ 全方向移动
- ✅ 原地旋转
- ✅ 斜向移动更高效
- ✅ 更好的转向能力

**控制因子**:
```cpp
Motor1 (左前45°):  throttle=1.0, steering=-1.0, lateral=-1.0
Motor2 (右前45°):  throttle=1.0, steering=1.0,  lateral=1.0
Motor3 (左后45°):  throttle=1.0, steering=-1.0, lateral=1.0
Motor4 (右后45°):  throttle=1.0, steering=1.0,  lateral=-1.0
```

**应用场景**:
- 竞赛机器人
- 快速响应AGV
- 精密装配平台

---

#### 🔹 FRAME_TYPE_OMNIPLUS (3) - +型全向

**布局**:
```
      前方
       ↑
       M1
       |
   M2--+--M3
       |
       M4

四个电机呈十字分布（0°/90°/180°/270°）
```

**运动能力**:
- ✅ 前后左右直线移动最优
- ✅ 原地旋转
- ✅ 对角线移动稍弱

**控制因子**:
```cpp
Motor1 (正前0°):   throttle=1.0,  steering=0.0,  lateral=0.0
Motor2 (左侧90°):  throttle=0.0,  steering=-1.0, lateral=-1.0
Motor3 (右侧90°):  throttle=0.0,  steering=1.0,  lateral=1.0
Motor4 (后方180°): throttle=-1.0, steering=0.0,  lateral=0.0
```

**应用场景**:
- 工业搬运机器人
- 仓库导引车
- 精确定位平台

---

#### 🔹 FRAME_TYPE_OMNI3MECANUM (4) - 三轮麦克纳姆

**布局**:
```
    前方
     ↑
   M1 [/]
   /    \
M2 [\]  [/] M3

三个麦克纳姆轮
特殊滚轮角度45°
```

**特点**:
- 麦克纳姆轮结构（带斜向滚轮）
- 三轮配置（非常规四轮）
- 全向移动能力
- 轮子磨损较快

**运动能力**:
- ✅ 全方向移动
- ✅ 平滑横移
- ⚠️ 承重能力略弱于标准轮

**应用场景**:
- 特殊狭窄空间作业
- 实验性机器人
- 教育演示

---

### 2.4 全向轮检测

**文件**: `libraries/AR_Motors/AP_MotorsUGV.h` (line 116)

```cpp
// returns true if the vehicle is omni
bool is_omni() const {
    return _frame_type != FRAME_TYPE_UNDEFINED && _motors_num > 0;
}
```

**条件**:
- `FRAME_TYPE` 不为 0（不是默认类型）
- `_motors_num > 0`（已配置电机）

---

## 3️⃣ 帆船 (Sailboat) 配置

### 3.1 帆船类定义

**文件**: `Rover/sailboat.h` (lines 19-119)

帆船不是一个独立的 FRAME_CLASS 或 FRAME_TYPE，而是 `FRAME_BOAT` 的一个特殊配置。

### 3.2 帆船启用检测

```cpp
class Sailboat
{
public:
    // enabled
    bool sail_enabled() const { return enable > 0; }

    // true if sailboat navigation (aka tacking) is enabled
    bool tack_enabled() const;
```

**启用条件**:
- `FRAME_CLASS = 2` (FRAME_BOAT)
- `SAIL_ENABLE > 0`

### 3.3 帆船类型（根据帆的类型）

#### 📐 传统主帆 (Mainsail)

**特点**:
- 单个主帆
- 帆角范围: 0-100%
- 通过舵机控制帆角

**参数**:
```cpp
AP_Float sail_angle_min;      // 最小帆角
AP_Float sail_angle_max;      // 最大帆角
AP_Float sail_angle_ideal;    // 理想帆角
```

**控制通道**:
- `RC_Channel *channel_mainsail` - 遥控主帆通道
- 输出函数: `SRV_Channel::k_mainsail`

**适用场景**:
- 传统帆船
- 单帆设计
- 简单风帆控制

---

#### 📐 翼帆 (Wingsail)

**特点**:
- 刚性机翼式帆
- 帆角范围: -100 to +100
- 可双向调整
- 更高效的升力

**控制**:
```cpp
void set_wingsail(float wingsail);  // -100 to +100
float get_wingsail() const;
```

**输出函数**: `SRV_Channel::k_wingsail`

**适用场景**:
- 高性能帆船
- 自动化程度高的USV
- 精确风向控制

---

#### 📐 桅杆旋转 (Mast Rotation)

**特点**:
- 控制桅杆转动
- 范围: -100 to +100
- 配合翼帆使用

**控制**:
```cpp
void set_mast_rotation(float mast_rotation);  // -100 to +100
float get_mast_rotation() const;
```

**输出函数**: `SRV_Channel::k_mast_rotation`

---

### 3.4 帆船电机辅助模式

**枚举**: `UseMotor` (sailboat.h lines 66-70)

```cpp
enum class UseMotor {
    USE_MOTOR_NEVER  = 0,  // 永不使用电机
    USE_MOTOR_ASSIST = 1,  // 辅助模式（低风速/转向时）
    USE_MOTOR_ALWAYS = 2   // 始终使用电机
};
```

#### 🔹 USE_MOTOR_NEVER (0)
- 纯风帆驱动
- 无电机辅助
- 完全依赖风力

#### 🔹 USE_MOTOR_ASSIST (1) - 智能辅助
- **低风速辅助**: `motor_assist_low_wind()`
  - 风速低于 `sail_windspeed_min` 时启动电机
- **转向辅助**: `motor_assist_tack()`
  - 抢风转向（tacking）时启动电机
  - 加快转向速度

#### 🔹 USE_MOTOR_ALWAYS (2)
- 电机始终运行
- 帆作为辅助推进
- 混合动力模式

**设置函数**:
```cpp
void set_motor_state(UseMotor state, bool report_failure = true);
```

**检测条件** (sailboat.cpp:518-521):
```cpp
if (rover.g2.motors.have_skid_steering() ||
    SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
    rover.get_frame_type() != rover.g2.motors.frame_type::FRAME_TYPE_UNDEFINED) {
    motor_state = state;  // 可以使用电机
}
```

### 3.5 帆船导航特性

#### 抢风转向 (Tacking)
- **用途**: 逆风行驶时的Z字形航线
- **触发**: 目标航向在禁航区（no-go zone）内
- **参数**:
  ```cpp
  AP_Float sail_no_go;  // 禁航角度（相对风向）
  ```

#### 速度指标 (VMG - Velocity Made Good)
```cpp
float get_VMG() const;
```
- 实际朝目标方向的有效速度
- 考虑风向和航向的夹角

#### 横向偏差控制
```cpp
AP_Float xtrack_max;  // 最大横向偏差
```

#### 舵角限制
```cpp
AP_Float sail_heel_angle_max;  // 最大倾斜角
```
- 防止倾覆
- 自动收帆

---

## 4️⃣ USV (本项目) 推荐配置

### 4.1 基本配置

```
FRAME_CLASS = 2        # FRAME_BOAT - 水面船只
FRAME_TYPE = 0         # 标准推进（单螺旋桨或差速推进）
```

### 4.2 动力配置选项

#### 选项 A: 单螺旋桨 + 舵机转向 ⭐ 推荐
```
FRAME_TYPE = 0
配置:
- SRV_Channel 1: k_steering (舵机)
- SRV_Channel 3: k_throttle (推进器)
```

**特点**:
- ✅ 结构简单
- ✅ 能效高
- ✅ 维护方便
- ⚠️ 低速转向能力弱

---

#### 选项 B: 差速双推进器
```
FRAME_TYPE = 0
配置:
- SRV_Channel 1: k_throttleLeft (左推进器)
- SRV_Channel 3: k_throttleRight (右推进器)
```

**特点**:
- ✅ 可原地转向
- ✅ 转向灵活
- ⚠️ 能耗较高
- ⚠️ 机械复杂度高

---

#### 选项 C: 矢量推进 (Vectored Thrust)
```
FRAME_TYPE = 0
MOT_VEC_ANGLEMAX > 0  # 设置最大矢量角度
配置:
- SRV_Channel 1: k_steering (控制推进器方向)
- SRV_Channel 3: k_throttle (推进器推力)
```

**特点**:
- ✅ 转向效率高
- ✅ 空间利用好
- ⚠️ 机械结构复杂
- ⚠️ 成本较高

---

### 4.3 帆船配置（如需要）

如果 USV 需要帆船功能（混合动力）:

```
FRAME_CLASS = 2        # FRAME_BOAT
FRAME_TYPE = 0         # 标准
SAIL_ENABLE = 1        # 启用帆船功能
SAIL_TYPE = 1          # 主帆类型

电机辅助:
USE_MOTOR = 1          # 辅助模式
SAIL_WNDSPD_MIN = 2.0  # 低于2m/s启用电机

配置通道:
- Channel X: k_mainsail (主帆控制)
- Channel 1: k_steering (舵机)
- Channel 3: k_throttle (辅助电机)
```

---

## 5️⃣ 参数配置指南

### 5.1 查看当前配置

**通过 MAVLink/Mission Planner**:
```
FRAME_CLASS?  # 查询框架类别
FRAME_TYPE?   # 查询框架类型
SAIL_ENABLE?  # 查询帆船启用状态
```

### 5.2 设置 USV 为标准船只

```
FRAME_CLASS 2   # 设置为船只
FRAME_TYPE 0    # 标准推进
SAIL_ENABLE 0   # 不使用帆（纯电机）
```

**重启后生效**: FRAME_TYPE 修改需要重启

### 5.3 检查配置是否生效

**MAVLink 消息**:
- `HEARTBEAT` 消息中的 `type` 字段应为 `11` (MAV_TYPE_SURFACE_BOAT)

**串口日志**:
```
ArduRover V4.x.x
Frame: BOAT
```

---

## 6️⃣ 代码中的关键检查

### 6.1 船只模式检查

**文件**: `Rover/system.cpp` (lines 321-324)

```cpp
bool Rover::is_boat() const
{
    return ((enum frame_class)g2.frame_class.get() == FRAME_BOAT);
}
```

**使用场景**:
- 导航算法选择
- 转向逻辑
- Loiter 模式行为
- 速度控制策略

### 6.2 全向轮检查

**文件**: `libraries/AR_Motors/AP_MotorsUGV.h` (line 116)

```cpp
bool is_omni() const {
    return _frame_type != FRAME_TYPE_UNDEFINED && _motors_num > 0;
}
```

### 6.3 帆船检查

**文件**: `Rover/sailboat.h` (line 27)

```cpp
bool sail_enabled() const { return enable > 0; }
```

---

## 7️⃣ 框架类型对行为的影响

### 7.1 导航行为差异

| 特性 | FRAME_ROVER | FRAME_BOAT | FRAME_BALANCEBOT |
|------|------------|-----------|-----------------|
| **原地转向** | ✅ 支持 | ❌ 不支持 | ✅ 支持 |
| **需要速度转向** | ❌ 不需要 | ✅ 需要 | ❌ 不需要 |
| **Loiter保持位置** | ✅ 精确 | ⚠️ 近似（漂移） | ✅ 精确 |
| **倒车能力** | ✅ 完全 | ⚠️ 受限 | ✅ 完全 |
| **俯仰角控制** | ❌ 不使用 | ❌ 不使用 | ✅ 核心 |

### 7.2 失效保护差异

**FRAME_BOAT 特殊处理**:
- RTL 模式保持适当速度（不停船）
- Loiter 模式允许漂移
- Hold 模式可能无法完全静止

**FRAME_ROVER**:
- Hold 模式完全停止
- Loiter 精确保持位置

---

## 8️⃣ 总结与建议

### 8.1 框架选择决策树

```
是否为水面载具？
├─ 是 → FRAME_CLASS = 2 (FRAME_BOAT)
│      │
│      ├─ 是否使用帆？
│      │  ├─ 是 → SAIL_ENABLE = 1
│      │  └─ 否 → SAIL_ENABLE = 0
│      │
│      └─ 推进类型？
│         ├─ 单推进器+舵机 → FRAME_TYPE = 0 ⭐
│         ├─ 双推进器差速 → FRAME_TYPE = 0
│         └─ 矢量推进 → FRAME_TYPE = 0 + MOT_VEC_ANGLEMAX
│
├─ 否，是否为平衡机器人？
│      ├─ 是 → FRAME_CLASS = 3 (FRAME_BALANCEBOT)
│      └─ 否 → FRAME_CLASS = 1 (FRAME_ROVER)
│             │
│             └─ 轮子配置？
│                ├─ 标准/差速/阿克曼 → FRAME_TYPE = 0
│                ├─ 三轮全向 → FRAME_TYPE = 1
│                ├─ X型全向 → FRAME_TYPE = 2
│                ├─ +型全向 → FRAME_TYPE = 3
│                └─ 三轮麦克纳姆 → FRAME_TYPE = 4
```

### 8.2 本项目 (ESP32-S3 USV) 推荐

```ini
# 基本配置
FRAME_CLASS = 2      # 水面船只
FRAME_TYPE = 0       # 标准推进
SAIL_ENABLE = 0      # 纯电机（无帆）

# 推进配置（选项A - 推荐）
# SRV1 = Steering (舵机)
# SRV3 = Throttle (推进电机)

# 如果是差速双推进器（选项B）
# SRV1 = ThrottleLeft
# SRV3 = ThrottleRight
```

**理由**:
- ✅ 水面运行环境
- ✅ 单推进器+舵机结构简单可靠
- ✅ 适合长时间巡航
- ✅ 功耗优化

### 8.3 关键参数验证清单

构建固件后验证:
- [ ] `FRAME_CLASS` 显示为 `2` (Boat)
- [ ] `FRAME_TYPE` 显示为 `0` (Default)
- [ ] MAVLink `HEARTBEAT.type` = `11` (MAV_TYPE_SURFACE_BOAT)
- [ ] 启动日志显示 "Frame: BOAT"
- [ ] Loiter 模式允许漂移（正常）
- [ ] Hold 模式尝试保持位置但可能漂移（正常）

---

## 📚 相关文件索引

### 定义文件
- `Rover/defines.h` (90-96) - FRAME_CLASS 枚举
- `libraries/AR_Motors/AP_MotorsUGV.h` (25-31) - FRAME_TYPE 枚举
- `Rover/sailboat.h` (66-70) - UseMotor 枚举

### 参数文件
- `Rover/Parameters.h` (328, 357) - 参数声明
- `Rover/Parameters.cpp` (419-424, 464-470) - 参数定义

### 实现文件
- `Rover/system.cpp` (321-324) - is_boat() 检查
- `Rover/balance_bot.cpp` (17-20) - is_balancebot() 检查
- `Rover/GCS_MAVLink_Rover.cpp` (10-16) - MAVLink 类型映射
- `Rover/sailboat.cpp` - 帆船实现
- `libraries/AR_Motors/AP_MotorsUGV.cpp` - 电机控制实现

---

**文档生成时间**: 2025-10-27
**适用固件版本**: ArduRover 4.x
**目标平台**: ESP32-S3 (USV)
