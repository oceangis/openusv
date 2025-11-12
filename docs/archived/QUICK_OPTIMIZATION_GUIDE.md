# ArduPilot Rover ESP32-S3 快速优化指南

## 快速决策表

### ✅ 可以立即移除（17个库）

| 库名称 | 移除原因 | 节省空间 | 风险 |
|--------|----------|----------|------|
| AP_LandingGear | 仅用于飞机/直升机 | ~15KB | 无 |
| AP_Parachute | Rover不需要降落伞 | ~18KB | 无 |
| AP_Soaring | 仅用于固定翼滑翔 | ~25KB | 无 |
| AP_TECS | 固定翼能量控制 | ~30KB | 无 |
| AP_VideoTX | FPV图传控制 | ~22KB | 无 |
| AP_IRLock | 红外跟踪 | ~12KB | 无 |
| AP_GyroFFT | 多旋翼滤波器 | ~35KB | 无 |
| AP_Quicktune | Copter自动调参 | ~20KB | 无 |
| AP_KDECAN | KDE电调 | ~18KB | 无 |
| AP_PiccoloCAN | Piccolo电调 | ~25KB | 无 |
| AP_BLHeli | BLHeli电调 | ~28KB | 无 |
| AP_FETtecOneWire | Fettec电调 | ~15KB | 无 |
| AP_RobotisServo | 机器人伺服 | ~12KB | 无 |
| AP_ADSB | 航空防撞 | ~32KB | 无 |
| AP_Generator | 发电机管理 | ~28KB | 无 |
| AP_EFI | 燃油喷射 | ~22KB | 无 |
| AP_Devo_Telem | Devo遥测 | ~8KB | 无 |
| **总计** | | **~365KB** | |

---

### ⚠️ 需要确认后移除（12个库）

| 库名称 | 确认问题 | 节省空间 | 建议 |
|--------|----------|----------|------|
| **AP_Torqeedo** | 是否使用Torqeedo电机？ | ~35KB | 如果不是Torqeedo，必须移除 |
| **AP_Camera** | 是否需要相机触发？ | ~40KB | 有监控需求建议保留 |
| **AP_Mount** | 是否有云台设备？ | ~45KB | 无云台可移除 |
| **AP_AIS** | 是否有AIS接收器？ | ~20KB | 有AIS硬件必须保留 |
| **AP_OpticalFlow** | 是否有光流传感器？ | ~30KB | GPS足够可移除 |
| **AP_Beacon** | 是否需要室内定位？ | ~25KB | 室外GPS足够 |
| **AC_PrecLand** | 是否需要精确对接？ | ~35KB | 无对接功能可移除 |
| **AP_Winch** | 是否有绞盘系统？ | ~18KB | 无绞盘可移除 |
| **AP_Follow** | 是否需要跟随功能？ | ~20KB | 无编队需求可移除 |
| **AC_Sprayer** | 是否需要喷洒功能？ | ~15KB | 无喷洒可移除 |
| **AP_Gripper** | 是否有夹持器？ | ~12KB | 无机械臂可移除 |
| **AP_Frsky_Telem** | 遥控器是否FrSky？ | ~25KB | 非FrSky可移除 |
| **总计（全部移除）** | | **~320KB** | |

---

### 🔧 可选移除（8个库）

| 库名称 | 用途 | 节省空间 | 权衡 |
|--------|------|----------|------|
| AP_Scripting | Lua脚本扩展 | ~150KB | 建议保留用于传感器集成 |
| AP_OSD | 视频屏显 | ~55KB | 无OSD需求可移除 |
| AP_Terrain | 地形数据库 | ~40KB | 无地形跟随可移除 |
| AP_OAPathPlanner | 高级避障 | ~30KB | 建议保留 |
| AP_WindVane | 风向标 | ~18KB | 气象站需要，建议保留 |
| AP_Hott_Telem | Hott遥测 | ~10KB | 非Hott可移除 |
| AP_LTM_Telem | LTM遥测 | ~6KB | 一般不用 |
| AP_Button | 物理按钮 | ~8KB | 无按钮可移除 |
| **总计（全部移除）** | | **~317KB** | |

---

## 三步实施方案

### 第一步：立即优化（5分钟）

编辑文件: `f:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\components\ardupilot\CMakeLists.txt`

在第 46 行之后（`list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_VisualOdom/.*\\.(c|cpp)$")`）添加：

```cmake
# ============================================================================
# 阶段1：移除明确不需要的库（安全优化）
# ============================================================================

# 飞行器类型专用库
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_LandingGear/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Parachute/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Soaring/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_TECS/.*\\.(c|cpp)$")

# 不适用传感器/设备
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_VideoTX/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_IRLock/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_GyroFFT/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Quicktune/.*\\.(c|cpp)$")

# 特殊硬件驱动（非通用电调）
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_KDECAN/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_PiccoloCAN/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_BLHeli/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_FETtecOneWire/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_RobotisServo/.*\\.(c|cpp)$")

# 航空特定功能
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_ADSB/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Generator/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_EFI/.*\\.(c|cpp)$")

# 不常用遥测协议
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Devo_Telem/.*\\.(c|cpp)$")
```

在第 303 行之后（`HAL_VISUALODOM_ENABLED=0`）添加：

```cmake
    # 阶段1优化：禁用明确不需要的功能
    HAL_PARACHUTE_ENABLED=0
    HAL_SOARING_ENABLED=0
    AP_VIDEOTX_ENABLED=0
    AP_IRLOCK_ENABLED=0
    HAL_ADSB_ENABLED=0
    HAL_GENERATOR_ENABLED=0
    HAL_EFI_ENABLED=0
```

**预计收益**: Flash节省 ~365KB

---

### 第二步：确认硬件配置

回答以下问题，确定是否可以继续优化：

#### 电机系统
- [ ] 使用 Torqeedo 电机？ → **是** 则保留 `AP_Torqeedo`
- [ ] 使用普通PWM/DroneCAN电机？ → **是** 则可移除 `AP_Torqeedo`

#### 相机系统
- [ ] 需要相机触发功能？ → **是** 则保留 `AP_Camera`
- [ ] 需要云台控制？ → **是** 则保留 `AP_Mount`

#### 传感器系统
- [ ] 有AIS船舶识别接收器？ → **是** 则保留 `AP_AIS`
- [ ] 有光流传感器？ → **否** 则移除 `AP_OpticalFlow`
- [ ] 需要室内无GPS定位？ → **否** 则移除 `AP_Beacon`

#### 功能需求
- [ ] 需要精确对接/停靠？ → **否** 则移除 `AC_PrecLand`
- [ ] 有绞盘/采样器？ → **否** 则移除 `AP_Winch`
- [ ] 需要跟随其他船？ → **否** 则移除 `AP_Follow`
- [ ] 有喷洒/投放设备？ → **否** 则移除 `AC_Sprayer`
- [ ] 有夹持器/机械臂？ → **否** 则移除 `AP_Gripper`

#### 遥测协议
- [ ] 遥控器品牌：____________
  - FrSky → 保留 `AP_Frsky_Telem`
  - Spektrum → 保留 `AP_RCTelemetry`（默认已启用）
  - 其他 → 移除 `AP_Frsky_Telem`, `AP_Hott_Telem`, `AP_LTM_Telem`

---

### 第三步：应用确认后的优化

根据第二步的回答，在 CMakeLists.txt 中添加（示例：假设都选"否"）：

```cmake
# ============================================================================
# 阶段2：基于硬件确认的优化
# ============================================================================

# 如果不使用 Torqeedo 电机
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Torqeedo/.*\\.(c|cpp)$")

# 如果不需要相机/云台
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Camera/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Mount/.*\\.(c|cpp)$")

# 如果无 AIS 接收器
# list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_AIS/.*\\.(c|cpp)$")  # 谨慎！

# 如果无光流传感器
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_OpticalFlow/.*\\.(c|cpp)$")

# 如果无室内定位需求
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Beacon/.*\\.(c|cpp)$")

# 如果不需要精确对接（从 ROVER_REQUIRED_AC_MODULES 中移除）
# 需要手动编辑第 55-60 行，移除 AC_PrecLand

# 如果无绞盘
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Winch/.*\\.(c|cpp)$")

# 如果不需要跟随
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Follow/.*\\.(c|cpp)$")

# 如果无夹持器
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Gripper/.*\\.(c|cpp)$")

# 如果不使用 FrSky 遥控
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Frsky_Telem/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Hott_Telem/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_LTM_Telem/.*\\.(c|cpp)$")
```

对应的 HAL 标志：

```cmake
    # 阶段2优化
    HAL_TORQEEDO_ENABLED=0
    AP_CAMERA_ENABLED=0
    HAL_MOUNT_ENABLED=0
    AP_OPTICALFLOW_ENABLED=0
    AP_BEACON_ENABLED=0
    AP_WINCH_ENABLED=0
    AP_FOLLOW_ENABLED=0
    AP_FRSKY_TELEM_ENABLED=0
```

**预计额外收益**: Flash节省 ~320KB

---

## 编译验证

```bash
cd f:/opensource/usv_esp32/esp32s3rover/ardupilot_rover_esp32s3_idf
idf.py fullclean
idf.py build
```

**检查输出**:
- ✅ 无编译错误
- ✅ 无链接错误（undefined reference）
- ✅ Flash使用量显著减少

---

## 功能测试清单

烧录固件后，验证以下功能：

### 基础功能
- [ ] MAVLink连接（地面站）
- [ ] 参数读写
- [ ] GPS定位
- [ ] 罗盘校准
- [ ] 电池监测

### DroneCAN
- [ ] CAN节点识别
- [ ] DroneCAN设备通信
- [ ] 参数配置

### 传感器
- [ ] DST800测深（UART2）
- [ ] 4G模块（UART4）
- [ ] 海流计（UART5）
- [ ] 水质仪（UART6）
- [ ] 气象站（UART7）

### 控制
- [ ] 手动模式
- [ ] 自动模式
- [ ] RTL返航
- [ ] 电机响应

### 日志
- [ ] 数据日志记录
- [ ] 日志下载

---

## 回退方案

如果出现问题，立即回退：

### 快速回退
注释掉添加的所有 `EXCLUDE REGEX` 行：
```cmake
# list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_XXX/.*\\.(c|cpp)$")
```

### 单个库恢复
只恢复有问题的库，例如：
```cmake
# 发现需要 AP_Camera，注释掉这行：
# list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Camera/.*\\.(c|cpp)$")
```

---

## 预期结果

### 阶段1（保守）
- Flash使用: 减少 ~365KB
- RAM使用: 减少 ~50KB
- 编译时间: 减少 15%
- 风险: 极低

### 阶段1+2（推荐）
- Flash使用: 减少 ~685KB
- RAM使用: 减少 ~120KB
- 编译时间: 减少 25%
- 风险: 低（需硬件确认）

### 完整优化
- Flash使用: 减少 ~1MB+
- RAM使用: 减少 ~200KB
- 编译时间: 减少 30%
- 风险: 中（需功能评估）

---

## 常见问题

### Q1: 移除库后编译报错 "undefined reference"
**A**: 该库被其他代码引用，需要保留。注释掉对应的 EXCLUDE 行。

### Q2: 功能缺失但无编译错误
**A**: 库被条件编译保护，检查 HAL_*_ENABLED 标志是否正确。

### Q3: 如何确认某库是否真的被使用？
**A**: 使用 grep 搜索：
```bash
cd f:/opensource/usv_esp32/ardupilot-master
grep -r "库名" Rover/
```

### Q4: DroneCAN是否依赖这些库？
**A**: DroneCAN核心功能不依赖这些可选库，但某些DroneCAN设备（如相机、云台）需要对应的库支持。

---

## 联系与支持

如有问题，请参考：
1. 详细分析报告: `ROVER_LIBRARY_DEEP_ANALYSIS.md`
2. ArduPilot文档: https://ardupilot.org/rover/
3. ESP32论坛: https://discuss.ardupilot.org/c/hardware-discussion/esp32/

---

**版本**: 1.0
**日期**: 2025-10-27
**适用项目**: ESP32-S3 USV Rover with DroneCAN
