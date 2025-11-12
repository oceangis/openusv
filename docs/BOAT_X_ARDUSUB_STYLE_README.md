# X型船 ArduSub风格控制改进总览

## 项目概述

本项目为 ESP32-S3 USV 的 X型船配置（FRAME_TYPE_BOAT_VECTORED_X）设计了基于 ArduSub VECTORED 风格的控制改进方案。

**改进目标**: 使 Rover 的3因子控制（throttle/steering/lateral）与 ArduSub 的6DOF控制逻辑一致。

---

## 快速导航

### 📖 文档列表

| 文档 | 用途 | 读者 |
|------|------|------|
| [深度对比分析](./ARDUSUB_CONTROL_ANALYSIS.md) | ArduSub vs Rover 控制架构详细分析 | 开发者 |
| [改进代码](./BOAT_X_IMPROVEMENT_CODE.md) | 具体代码修改和实施步骤 | 开发者 |
| [测试指南](./BOAT_X_TEST_GUIDE.md) | 完整测试流程和验证方法 | 测试人员 |
| **本文档** | 总览和快速开始 | 所有人 |

---

## 核心问题

### 当前问题（修改前）

在 X型船配置下，执行 **纯横移命令**（throttle=0, steering=0, lateral=+100）时：

```
船体行为：横移 + 旋转（错误！）
原因：lateral因子不对称

M1 (左前): lateral = -0.707 → 推
M2 (右前): lateral = +0.707 → 拉
M3 (左后): lateral = +0.707 → 拉
M4 (右后): lateral = -0.707 → 推

结果：形成对角线推拉 → 产生旋转力矩
```

### 解决方案（修改后）

**仅修改2行代码**，修正M1和M2的lateral因子：

```cpp
// 修改前
add_omni_motor(0, cos45, -cos45, -cos45);  // M1
add_omni_motor(1, cos45, cos45, cos45);    // M2

// 修改后
add_omni_motor(0, cos45, -cos45, cos45);   // M1 ← lateral符号翻转
add_omni_motor(1, cos45, cos45, -cos45);   // M2 ← lateral符号翻转
```

**修改后的行为**：

```
M1 (左前): lateral = +0.707 → 推
M2 (右前): lateral = -0.707 → 拉
M3 (左后): lateral = +0.707 → 推
M4 (右后): lateral = -0.707 → 拉

结果：左侧(M1+M3)推，右侧(M2+M4)拉 → 纯横移！
```

---

## 5分钟快速开始

### 1. 备份和修改代码（2分钟）

```bash
cd F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf

# 备份
cp libraries/AR_Motors/AP_MotorsUGV.cpp libraries/AR_Motors/AP_MotorsUGV.cpp.backup

# 编辑文件
# 找到行727和730，修改lateral因子（第3个参数）
# 行727: add_omni_motor(0, cos45, -cos45, -cos45);
#   改为: add_omni_motor(0, cos45, -cos45, cos45);
# 行730: add_omni_motor(1, cos45, cos45, cos45);
#   改为: add_omni_motor(1, cos45, cos45, -cos45);
```

### 2. 编译和刷写（3分钟）

```bash
# 编译
D:\Espressif\v5.5.1\esp-idf\tools\idf.py build

# 刷写
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM3 flash
```

### 3. 测试验证（10分钟）

**关键测试**：Pure Lateral Right

```
步骤:
1. 设置 FRAME_TYPE=11
2. 重启ESP32
3. 进入Manual模式
4. 推杆: Throttle=0, Steering=0, Lateral=+100%
5. 观察船体运动

预期结果:
✅ 船体纯横移向右，无旋转（航向偏差 < ±5°）
❌ 如果有旋转，说明修改未生效

如果成功:
- 航向保持稳定
- GPS航迹纯横向移动
- 无前进/后退运动
```

---

## 技术细节

### 因子对比表

| Motor | 位置 | Throttle | Steering | Lateral (修改前) | Lateral (修改后) |
|-------|------|----------|----------|-----------------|-----------------|
| M1    | 左前 | +0.707   | -0.707   | **-0.707**      | **+0.707** ✅   |
| M2    | 右前 | +0.707   | +0.707   | **+0.707**      | **-0.707** ✅   |
| M3    | 左后 | -0.707   | -0.707   | +0.707          | +0.707         |
| M4    | 右后 | -0.707   | +0.707   | -0.707          | -0.707         |

### ArduSub VECTORED 映射

| Rover | 位置 | ArduSub对应 | Forward | Yaw  | Lateral |
|-------|------|------------|---------|------|---------|
| M1    | 左前 | M3         | +1      | -1   | +1      |
| M2    | 右前 | M4         | +1      | +1   | -1      |
| M3    | 左后 | M1         | -1      | +1   | +1      |
| M4    | 右后 | M2         | -1      | -1   | -1      |

**映射关系**：Rover因子 = ArduSub因子 * cos(45°) = ArduSub因子 * 0.707

---

## 设计原则

### 方案选择：方案A（推荐）

**保持3因子，优化因子计算**

✅ **优点**:
- 最小化修改（仅2行）
- 不破坏Rover架构
- 保持计算效率
- ESP32-S3资源友好
- 风险可控

❌ **方案B（不推荐）**: 扩展为6因子，模拟ArduSub
- 需要500+行代码修改
- 破坏Rover架构稳定性
- 性能损失（双循环计算）
- 测试复杂度高

### 控制映射

```
Rover 3因子           ArduSub 6DOF
┌──────────┐         ┌──────────┐
│ throttle │────────→│ forward  │  前后运动
│ steering │────────→│ yaw      │  偏航旋转
│ lateral  │────────→│ lateral  │  横向平移
└──────────┘         └──────────┘
                      │ throttle │  垂直（USV不用）
                      │ roll     │  翻滚（USV不用）
                      │ pitch    │  俯仰（USV不用）
                      └──────────┘
```

---

## 测试清单

### 必须通过的测试

- [ ] **Test 2.3**: Pure Lateral Right（航向偏差 < ±5°）← **最关键！**
- [ ] Test 2.1: Pure Forward（航向偏差 < ±5°）
- [ ] Test 2.2: Pure Yaw（GPS位移 < 1m）
- [ ] Test 2.4: Pure Lateral Left（航向偏差 < ±5°）
- [ ] Test 3.2: Diagonal Movement（沿45°直线）

### 测试数据示例

**Test 2.3 成功示例**:
```
命令: Throttle=0, Steering=0, Lateral=+100
电机输出:
  M1: +70.7%
  M2: -70.7%
  M3: +70.7%
  M4: -70.7%
船体运动:
  方向: 纯右移
  航向变化: +2° (< ±5°) ✅
  GPS横移: 5.2m
结论: PASS
```

---

## FAQ

### Q1: 为什么只修改M1和M2？
**A**: M3和M4的lateral因子本来就是正确的（+0.707和-0.707），只有M1和M2符号错误。

### Q2: 修改后会影响Forward和Yaw吗？
**A**: 不会。只修改了lateral因子（第3个参数），throttle和steering因子（第1、2个参数）保持不变。

### Q3: 如何验证修改是否成功？
**A**: 执行 Test 2.3 (Pure Lateral Right)，如果船体纯横移无旋转，说明成功。

### Q4: 如果测试失败怎么办？
**A**:
1. 检查代码是否正确修改（grep检查）
2. 重新编译和刷写
3. 检查FRAME_TYPE是否设置为11
4. 查看测试指南的故障排查章节

### Q5: 修改后需要重新调参吗？
**A**: 可能需要微调Lateral PID，因为运动特性改变了。但Forward和Yaw的PID不需要改。

---

## 文件清单

### 源代码修改
```
libraries/AR_Motors/AP_MotorsUGV.cpp
  行727: M1的lateral因子  -0.707 → +0.707
  行730: M2的lateral因子  +0.707 → -0.707
```

### 文档
```
docs/
├── ARDUSUB_CONTROL_ANALYSIS.md      # 深度分析（技术细节）
├── BOAT_X_IMPROVEMENT_CODE.md       # 代码修改（实施指南）
├── BOAT_X_TEST_GUIDE.md             # 测试验证（测试手册）
└── BOAT_X_ARDUSUB_STYLE_README.md   # 本文档（总览）
```

---

## 预期收益

### 功能改进
- ✅ 纯横移运动（无旋转）
- ✅ 对角线运动效率最高（X型优势）
- ✅ 与ArduSub控制逻辑一致
- ✅ 更直觉的控制体验

### 代码质量
- ✅ 因子映射清晰（详细注释）
- ✅ 与ROV社区标准一致
- ✅ 易于理解和维护

### 性能
- ✅ 无性能损失（仅修改因子数值）
- ✅ 计算效率不变
- ✅ 内存占用不变

---

## 风险评估

### 低风险 ✅
- **代码改动小**: 仅2行数值修改
- **不影响其他配置**: 仅FRAME_TYPE=11受影响
- **易于回滚**: 备份文件或手动改回

### 中等风险 ⚠️
- **Lateral方向变化**: 已使用旧版本的用户需要适应
- **PID需要微调**: Lateral PID参数可能需要调整

### 缓解措施
1. 详细文档和测试指南
2. 备份原始代码
3. 渐进式测试（从小百分比开始）
4. 提供回滚步骤

---

## 版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| v1.0 | 2025-11-09 | 初始版本：lateral因子修正 |

---

## 参考资料

### ArduPilot官方文档
- [ArduSub Motor Setup](https://www.ardusub.com/operators-manual/motor-setup.html)
- [Rover Frame Types](https://ardupilot.org/rover/docs/rover-motor-and-servo-configuration.html)

### 源代码
- ArduSub 6DOF: `F:\opensource\usv_esp32\ardupilot-master\libraries\AP_Motors\AP_Motors6DOF.cpp`
- Rover UGV: `libraries\AR_Motors\AP_MotorsUGV.cpp`

### 相关项目
- ArduRemoteID (DroneCAN纯C接口参考): `F:\opensource\usv_esp32\ArduRemoteID-master`

---

## 联系和支持

### 问题反馈
- GitHub Issues: (待添加)
- 邮件: (待添加)

### 贡献
欢迎贡献代码、文档和测试报告！

---

**文档版本**: v1.0
**作者**: ArduPilot ESP32-S3 Team
**日期**: 2025-11-09
**许可**: GPL v3.0
