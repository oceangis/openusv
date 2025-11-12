# X型船 PosHold 和 Manual 航向辅助使用指南

## 概述

本文档描述如何使用为X型矢量推进船（FRAME_TYPE_BOAT_VECTORED_X = 11）新增的功能：
1. **PosHold模式** - GPS定位定向模式
2. **Manual模式航向辅助** - ArduSub风格的航向保持

这些功能专门为无人船表演设计，提供精确的定位定向控制。

---

## 一、基础配置

### 1. 设置船型为X型

```ini
FRAME_CLASS = 2          # 水面船只
FRAME_TYPE = 11          # BoatVectoredX (X型矢量推进)
```

### 2. 电机通道映射

```ini
# X型配置的4个45°推进器
SERVO1_FUNCTION = 73     # ThrottleLeft (Motor 1: 左前)
SERVO2_FUNCTION = 74     # ThrottleRight (Motor 2: 右前)
SERVO3_FUNCTION = 75     # Throttle (Motor 3: 左后)
SERVO4_FUNCTION = 76     # Lateral (Motor 4: 右后)
```

### 3. 遥控器通道映射

```ini
# 至少需要5个通道
RC1 = Roll (暂不使用)
RC2 = Pitch/Throttle (前进/后退)
RC3 = Throttle (油门 - 通常与RC2相同)
RC4 = Yaw (旋转)
RC5 = Mode (模式切换)
RC6 = Lateral (横移，可选)
```

---

## 二、PosHold 模式 (模式13)

### 功能特性

**PosHold**模式提供ArduSub风格的定位定向控制：

✅ **GPS位置保持** - 无摇杆输入时保持当前GPS位置
✅ **Body坐标系输入** - 前进/侧向摇杆相对船体方向移动
✅ **独立航向保持** - 250ms平滑减速 + 绝对角度锁定
✅ **三轴独立控制** - 可同时前进、侧移、旋转

### 使用方法

#### 1. 切换到PosHold模式

```
MODE = 13     # 通过地面站或遥控器切换
```

#### 2. 摇杆控制

| 摇杆输入 | 功能 | 说明 |
|---------|------|------|
| **Throttle (前后)** | 前进/后退 | 前推前进，后拉后退，松开后保持位置 |
| **Yaw (左右旋转)** | 旋转 | 左/右旋转，松开后锁定航向 |
| **Lateral (横移)** | 侧移 | 向左/右平移，保持航向不变 |
| **无输入** | 定点悬停 | 自动保持GPS位置和航向 |

#### 3. 航向保持逻辑

**ArduSub 250ms平滑减速**：
- **有Yaw输入时**：角速率控制，船体旋转
- **松开Yaw后前250ms**：平滑减速到0（无超调）
- **250ms后**：锁定绝对航向（精度±5°）

#### 4. 位置保持逻辑

- **有前向/侧向输入**：按摇杆方向移动，位置目标随之更新
- **松开所有摇杆**：使用GPS反馈保持当前位置
  - 位置控制：P控制器，误差转速度
  - 速度控制：PID闭环，输出电机油门

### 配置参数

```ini
# 速度控制
CRUISE_SPEED = 2.0         # 最大前进速度 (m/s)
ATC_SPEED_P = 0.20         # 速度控制P增益
ATC_SPEED_I = 0.20         # 速度控制I增益

# 横移速度控制 (新增)
ATC_LAT_SPD_P = 0.30       # 横移速度P增益
ATC_LAT_SPD_I = 0.15       # 横移速度I增益
ATC_LAT_SPD_IMAX = 1.00    # 横移I项限幅

# 位置保持
LOIT_SPEED_GAIN = 1.0      # 位置误差到速度的增益

# 航向控制
ATC_STR_ANG_P = 2.0        # 航向角度P增益
ATC_STR_RAT_P = 0.20       # 转向率P增益
ATC_STR_RAT_I = 0.20       # 转向率I增益
ACRO_TURN_RATE = 90        # 最大旋转速度 (度/秒)
```

### 表演动作示例

#### 1. 原地旋转
```
1. 松开所有摇杆（船体定点悬停）
2. 推动Yaw摇杆（360°旋转）
3. 松开Yaw摇杆（停止旋转，保持航向）
预期：位置偏移 ≤ 1米，航向精确控制
```

#### 2. 纯侧移
```
1. 松开所有摇杆
2. 推动Lateral摇杆（侧向平移5米）
3. 松开Lateral摇杆（停止并保持位置）
预期：航向保持±5°，纯横向移动
```

#### 3. 对角线移动
```
1. 同时推动Throttle和Lateral摇杆（45°方向移动）
2. 松开摇杆（停止并保持位置+航向）
预期：速度矢量合成正确，无相互干扰
```

#### 4. 组合动作
```
1. 前进 + 侧移 + 旋转同时进行
2. 松开所有摇杆（定点悬停）
预期：三轴独立控制，动作流畅
```

---

## 三、Manual 模式航向辅助

### 功能特性

**Manual模式**在X型船上自动启用航向辅助：

✅ **自动检测** - 检测到X型船时自动启用
✅ **航向保持** - 与PosHold相同的250ms平滑逻辑
✅ **手动油门/侧向** - 油门和侧向通道完全手动
✅ **不影响其他船型** - 仅X型船启用，其他船型保持原有行为

### 使用方法

#### 1. 切换到Manual模式

```
MODE = 0     # Manual模式
```

#### 2. 摇杆控制

| 摇杆输入 | 功能 | 说明 |
|---------|------|------|
| **Throttle** | 前进/后退 | 直接控制前向油门（无速度闭环） |
| **Yaw** | 旋转 | 航向保持辅助（250ms平滑） |
| **Lateral** | 侧移 | 直接控制横移油门（无速度闭环） |

#### 3. 与传统Manual模式的区别

**X型船 (FRAME_TYPE = 11)**：
- Yaw通道：航向保持辅助（ArduSub风格）
- Throttle/Lateral：直通控制（无PID）

**其他船型**：
- 所有通道：完全直通（传统Manual模式）
- 无任何辅助功能

### 典型应用场景

**表演直线行驶**：
```
1. Manual模式
2. 推动Throttle摇杆（船体前进）
3. 松开Yaw摇杆（自动保持航向）
预期：船体走直线，航向偏差≤±5°
```

**横移表演**：
```
1. Manual模式
2. 推动Lateral摇杆（横移）
3. Yaw自动保持航向
预期：纯横向移动，无旋转
```

---

## 四、模式对比表

| 特性 | Manual (X型) | PosHold | Loiter |
|------|-------------|---------|--------|
| **位置保持** | ❌ 无 | ✅ GPS闭环 | ✅ 简单P控制 |
| **航向保持** | ✅ 250ms平滑 | ✅ 250ms平滑 | ⚠️ 绑定目标方向 |
| **速度闭环** | ❌ 无 | ✅ 前向+横移 | ✅ 仅前向 |
| **横移控制** | ✅ 直通 | ✅ PID闭环 | ❌ 无 |
| **GPS需求** | ❌ 不需要 | ✅ 必须 | ✅ 必须 |
| **适用场景** | 手动飞行/调试 | 表演/定点作业 | 简单导航 |

---

## 五、调试和优化

### 1. 航向保持调试

**问题：航向振荡**
```ini
# 减小P增益
ATC_STR_RAT_P = 0.15   # 从0.20降低到0.15
```

**问题：航向响应慢**
```ini
# 增大P增益
ATC_STR_RAT_P = 0.25   # 从0.20增加到0.25
```

**问题：航向有稳态误差**
```ini
# 增大I增益
ATC_STR_RAT_I = 0.30   # 从0.20增加到0.30
```

### 2. 位置保持调试 (PosHold)

**问题：位置振荡**
```ini
# 减小位置P增益
LOIT_SPEED_GAIN = 0.5   # 从1.0降低到0.5
```

**问题：位置漂移（有流水）**
```ini
# 增大速度I增益
ATC_SPEED_I = 0.30      # 从0.20增加到0.30
ATC_LAT_SPD_I = 0.20    # 从0.15增加到0.20
```

**问题：位置回正慢**
```ini
# 增大位置P增益
LOIT_SPEED_GAIN = 1.5   # 从1.0增加到1.5
```

### 3. 横移控制调试

**问题：横移无力**
```ini
# 增大横移P增益
ATC_LAT_SPD_P = 0.40    # 从0.30增加到0.40
```

**问题：横移振荡**
```ini
# 减小横移P增益，增加滤波
ATC_LAT_SPD_P = 0.20
ATC_LAT_SPD_FILT = 15.0  # 从10增加到15 Hz
```

### 4. 日志分析

**关键日志消息**：
```
GPS    - GPS位置和速度
ATT    - 姿态（航向）
RCOU   - 电机输出 (C1-C4)
CTUN   - 控制器调优数据
```

**MAVExplorer分析**：
```python
# 航向跟踪
graph ATT.Yaw

# 位置误差（需要在代码中添加日志）
graph GPS.Lat GPS.Lng

# 电机输出
graph RCOU.C1 RCOU.C2 RCOU.C3 RCOU.C4
```

---

## 六、性能指标

### 预期性能 (GPS模式)

| 指标 | 目标值 | RTK-GPS |
|------|--------|---------|
| **静水定点精度 (CEP50)** | ≤ 1.0m | ≤ 0.3m |
| **流水定点精度 (0.3m/s流速)** | ≤ 2.0m | ≤ 0.5m |
| **航向保持精度** | ±5° | ±5° |
| **航向回正时间** | ≤ 3秒 | ≤ 3秒 |
| **最大前向速度** | 2.0 m/s | 2.0 m/s |
| **最大横移速度** | 1.5 m/s | 1.5 m/s |

### 测试清单

- [ ] Manual模式航向保持测试
- [ ] PosHold模式定点悬停测试（静水）
- [ ] PosHold模式定点悬停测试（流水）
- [ ] 原地旋转测试（位置偏移≤1米）
- [ ] 纯侧移测试（航向保持±5°）
- [ ] 对角线移动测试
- [ ] 组合动作测试（前进+侧移+旋转）
- [ ] GPS丢失恢复测试
- [ ] 电池低电量RTL测试

---

## 七、常见问题

### Q1: PosHold模式无法进入
**A**: 检查：
1. `FRAME_TYPE = 11` (BoatVectoredX)
2. GPS lock（卫星≥6颗）
3. 地面站显示：GPS.Status ≥ 3

### Q2: 航向保持不工作
**A**: 检查：
1. 罗盘校准
2. `ATC_STR_RAT_P/I` 参数是否为0
3. 电机输出是否正常（RCOU日志）

### Q3: 位置漂移严重
**A**: 可能原因：
1. GPS精度差（增加RTK模块）
2. 有流水扰动（增加ATC_SPEED_I）
3. 位置P增益太小（增加LOIT_SPEED_GAIN）

### Q4: 横移不工作
**A**: 检查：
1. Lateral通道映射（RC6 → Lateral）
2. `ATC_LAT_SPD_P/I` 参数
3. 电机4输出（RCOU.C4）

### Q5: Manual模式无航向辅助
**A**: 确认：
1. `FRAME_TYPE = 11`（只有X型船启用）
2. 其他船型保持传统Manual模式（这是正常的）

---

## 八、技术参考

### 代码文件位置

**PosHold模式**：
- `Rover/mode_poshold.cpp` - 模式实现
- `Rover/mode.h:669` - ModePosHold类声明

**Manual航向辅助**：
- `Rover/mode_manual.cpp:52` - update_heading_assist()
- `Rover/mode.h:737` - 航向辅助成员变量

**横移速度控制**：
- `libraries/APM_Control/AR_AttitudeControl.cpp:1156` - get_lateral_speed()
- `libraries/APM_Control/AR_AttitudeControl.cpp:1177` - get_lateral_out_speed()
- `libraries/APM_Control/AR_AttitudeControl.cpp:1199` - get_vectored_out_speed()

### ArduSub参考

这些功能基于ArduSub的以下模式：
- ArduSub/mode_poshold.cpp - Position Hold
- ArduSub/mode_stabilize.cpp - 250ms航向平滑逻辑
- libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.cpp - 6DOF控制

---

## 九、版本信息

- **实施日期**: 2025-11-09
- **ArduPilot版本**: Rover ESP32-S3
- **适用船型**: FRAME_TYPE_BOAT_VECTORED_X (11)
- **依赖**: GPS, 罗盘, ESP32-S3

---

## 联系和支持

如有问题或建议，请参考：
- Agent分析报告：`docs/ARDUSUB_CONTROL_ANALYSIS.md`
- X型船实现：`docs/BOAT_X_IMPROVEMENT_CODE.md`
- 测试指南：`docs/BOAT_X_TEST_GUIDE.md`

**祝您表演成功！** 🎉
