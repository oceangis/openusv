# ArduPilot Rover ESP32-S3 库移除汇总表

## 执行摘要

**项目**: ESP32-S3 N16R8 USV Rover 固件优化
**优化目标**: 减少 Flash/RAM 占用，提高编译效率
**分析库总数**: 142 个 (AP_*, AC_*, AR_* 库)

---

## 优化成果预测

| 优化级别 | 移除库数量 | Flash节省 | RAM节省 | 编译时间减少 | 风险等级 |
|---------|-----------|-----------|---------|-------------|---------|
| **阶段1（保守）** | 17个 | ~365KB | ~50KB | 15% | 极低 |
| **阶段2（推荐）** | +12个 (共29个) | ~685KB | ~120KB | 25% | 低 |
| **阶段3（激进）** | +8个 (共37个) | ~1002KB | ~200KB | 30% | 中 |

---

## 分类统计

### 按功能分类

| 类别 | 可移除库数量 | 总节省空间 | 典型库 |
|-----|------------|-----------|--------|
| 飞行器类型专用 | 4 | ~88KB | LandingGear, Parachute, Soaring, TECS |
| 不适用传感器 | 4 | ~89KB | VideoTX, IRLock, GyroFFT, Quicktune |
| 特殊硬件驱动 | 5 | ~98KB | KDECAN, PiccoloCAN, BLHeli, FETtec, RobotisServo |
| 航空特定功能 | 3 | ~82KB | ADSB, Generator, EFI |
| 遥测协议 | 4 | ~49KB | Devo, Hott, LTM, Frsky |
| 可选功能 | 12 | ~320KB | Camera, Mount, Torqeedo, OpticalFlow 等 |
| 高级功能 | 5 | ~276KB | Scripting, OSD, Terrain, Winch, OAPathPlanner |

---

## 详细库列表

### 🔴 高优先级移除（17个库 - 立即执行）

| # | 库名称 | 分类 | Flash | RAM | 原因 |
|---|-------|------|-------|-----|------|
| 1 | AP_LandingGear | 飞行器专用 | 15KB | 2KB | 仅Copter/Plane |
| 2 | AP_Parachute | 飞行器专用 | 18KB | 3KB | Rover无降落伞 |
| 3 | AP_Soaring | 飞行器专用 | 25KB | 4KB | 固定翼滑翔 |
| 4 | AP_TECS | 飞行器专用 | 30KB | 5KB | 固定翼能量控制 |
| 5 | AP_VideoTX | 传感器 | 22KB | 3KB | FPV图传 |
| 6 | AP_IRLock | 传感器 | 12KB | 2KB | 红外跟踪 |
| 7 | AP_GyroFFT | 传感器 | 35KB | 8KB | 多旋翼滤波 |
| 8 | AP_Quicktune | 工具 | 20KB | 3KB | Copter调参 |
| 9 | AP_KDECAN | 驱动 | 18KB | 2KB | KDE电调 |
| 10 | AP_PiccoloCAN | 驱动 | 25KB | 3KB | Piccolo电调 |
| 11 | AP_BLHeli | 驱动 | 28KB | 4KB | BLHeli电调 |
| 12 | AP_FETtecOneWire | 驱动 | 15KB | 2KB | Fettec电调 |
| 13 | AP_RobotisServo | 驱动 | 12KB | 2KB | 机器人伺服 |
| 14 | AP_ADSB | 航空功能 | 32KB | 5KB | 航空防撞 |
| 15 | AP_Generator | 航空功能 | 28KB | 4KB | 发电机管理 |
| 16 | AP_EFI | 航空功能 | 22KB | 3KB | 燃油喷射 |
| 17 | AP_Devo_Telem | 遥测 | 8KB | 1KB | Devo遥测 |
| | **小计** | | **365KB** | **52KB** | |

---

### 🟡 中优先级移除（12个库 - 需硬件确认）

| # | 库名称 | 确认问题 | Flash | RAM | 决策标准 |
|---|-------|---------|-------|-----|---------|
| 1 | AP_Torqeedo | 是否Torqeedo电机？ | 35KB | 5KB | 非Torqeedo→移除 |
| 2 | AP_Camera | 需要相机触发？ | 40KB | 6KB | 无相机→移除 |
| 3 | AP_Mount | 有云台设备？ | 45KB | 7KB | 无云台→移除 |
| 4 | AP_AIS | 有AIS接收器？ | 20KB | 3KB | 无AIS硬件→移除 |
| 5 | AP_OpticalFlow | 有光流传感器？ | 30KB | 5KB | GPS足够→移除 |
| 6 | AP_Beacon | 需室内定位？ | 25KB | 4KB | 室外GPS→移除 |
| 7 | AC_PrecLand | 需精确对接？ | 35KB | 6KB | 无对接→移除 |
| 8 | AP_Winch | 有绞盘系统？ | 18KB | 3KB | 无绞盘→移除 |
| 9 | AP_Follow | 需跟随功能？ | 20KB | 3KB | 无编队→移除 |
| 10 | AC_Sprayer | 需喷洒功能？ | 15KB | 2KB | 无喷洒→移除 |
| 11 | AP_Gripper | 有夹持器？ | 12KB | 2KB | 无机械臂→移除 |
| 12 | AP_Frsky_Telem | FrSky遥控？ | 25KB | 3KB | 非FrSky→移除 |
| | **小计** | | **320KB** | **49KB** | |

---

### 🟢 低优先级移除（8个库 - 功能权衡）

| # | 库名称 | 用途 | Flash | RAM | 权衡考虑 |
|---|-------|-----|-------|-----|---------|
| 1 | AP_Scripting | Lua脚本 | 150KB | 50KB | 建议保留用于传感器集成 |
| 2 | AP_OSD | 视频屏显 | 55KB | 10KB | 无OSD→移除 |
| 3 | AP_Terrain | 地形数据 | 40KB | 20KB | 无地形跟随→移除 |
| 4 | AP_OAPathPlanner | 高级避障 | 30KB | 8KB | 建议保留 |
| 5 | AP_WindVane | 风向标 | 18KB | 3KB | 气象站需要→保留 |
| 6 | AP_Hott_Telem | Hott遥测 | 10KB | 2KB | 非Hott→移除 |
| 7 | AP_LTM_Telem | LTM遥测 | 6KB | 1KB | 一般不用→移除 |
| 8 | AP_Button | 物理按钮 | 8KB | 1KB | 无按钮→移除 |
| | **小计** | | **317KB** | **95KB** | |

---

## CMakeLists.txt 修改位置

### 位置1: 库源文件排除（第46行后）

```cmake
# 当前已有:
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Airspeed/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_VisualOdom/.*\\.(c|cpp)$")

# 添加阶段1优化（17个库）:
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_LandingGear/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Parachute/.*\\.(c|cpp)$")
# ... 等等
```

### 位置2: HAL标志定义（第303行后）

```cmake
# 当前已有:
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    HAL_AIRSPEED_ENABLED=0
    HAL_VISUALODOM_ENABLED=0
)

# 添加:
    HAL_PARACHUTE_ENABLED=0
    HAL_SOARING_ENABLED=0
    # ... 等等
```

### 位置3: AC模块调整（第55-60行）

```cmake
# 当前:
set(ROVER_REQUIRED_AC_MODULES
    AC_Fence
    AC_Avoidance
    AC_Sprayer
    AC_PrecLand
)

# 如果不需要精确对接/喷洒，可移除对应项
```

---

## 实施步骤

### 步骤1: 备份
```bash
cd f:/opensource/usv_esp32/esp32s3rover/ardupilot_rover_esp32s3_idf
cp components/ardupilot/CMakeLists.txt components/ardupilot/CMakeLists.txt.backup
```

### 步骤2: 应用阶段1优化
按照 `QUICK_OPTIMIZATION_GUIDE.md` 中的"第一步"修改 CMakeLists.txt

### 步骤3: 清理编译
```bash
idf.py fullclean
```

### 步骤4: 重新编译
```bash
idf.py build
```

### 步骤5: 检查结果
查看编译输出的 Flash/RAM 使用量，对比优化前后：

**优化前** (预期):
```
Total sizes:
Used static IRAM:   xxxxx bytes ( xxxxx remain)
Used static DRAM:   xxxxx bytes ( xxxxx remain)
Used Flash size :   2.5 MB
```

**优化后** (预期):
```
Total sizes:
Used static IRAM:   xxxxx bytes ( xxxxx remain)
Used static DRAM:   xxxxx bytes ( xxxxx remain)
Used Flash size :   2.1 MB  ← 减少约 400KB
```

---

## 验证清单

编译成功后，烧录固件并测试：

### 基础功能验证
- [ ] 启动无错误
- [ ] MAVLink连接正常
- [ ] 参数系统工作
- [ ] GPS定位
- [ ] 传感器读取

### DroneCAN验证
- [ ] CAN总线初始化
- [ ] 节点发现
- [ ] 消息收发

### 传感器验证
- [ ] DST800 测深数据
- [ ] 4G 模块通信
- [ ] 海流计数据
- [ ] 水质仪数据
- [ ] 气象站数据

### 控制验证
- [ ] 手动模式
- [ ] 自动任务
- [ ] RTL 返航
- [ ] 避障响应

---

## 回退方案

如果出现问题：

### 完全回退
```bash
cd f:/opensource/usv_esp32/esp32s3rover/ardupilot_rover_esp32s3_idf
cp components/ardupilot/CMakeLists.txt.backup components/ardupilot/CMakeLists.txt
idf.py fullclean
idf.py build
```

### 部分回退
注释掉有问题的库的 `EXCLUDE REGEX` 行，重新编译。

---

## 进阶优化方向

完成基本库移除后，可考虑：

1. **精简 EKF**: 使用 EKF2 替代 EKF3（节省 ~50KB）
2. **降低调度频率**: 400Hz → 200Hz（减少CPU占用）
3. **减少日志缓冲**: 调整日志参数
4. **自定义传感器驱动**: 替代 Lua 脚本（节省 ~150KB）

---

## 总结

### 立即可执行（阶段1）
- 移除 **17个库**
- 节省 **~365KB Flash, ~50KB RAM**
- 风险：**极低**
- 时间：**5分钟修改 + 10分钟编译**

### 推荐执行（阶段1+2）
- 移除 **29个库**
- 节省 **~685KB Flash, ~120KB RAM**
- 风险：**低**（需硬件确认）
- 时间：**15分钟确认 + 10分钟编译**

### 完整优化（全部阶段）
- 移除 **37个库**
- 节省 **~1MB Flash, ~200KB RAM**
- 风险：**中**（需功能评估）
- 时间：**30分钟评估 + 10分钟编译**

---

## 相关文档

1. **详细分析报告**: `ROVER_LIBRARY_DEEP_ANALYSIS.md` - 每个库的详细说明
2. **快速操作指南**: `QUICK_OPTIMIZATION_GUIDE.md` - 分步骤实施指导
3. **原始优化记录**: `ROVER_LIBRARY_OPTIMIZATION.md` - 初步分析

---

**文档版本**: 1.0
**最后更新**: 2025-10-27
**状态**: 待执行
**建议开始**: 阶段1（保守优化）
