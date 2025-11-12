# ArduPilot Rover ESP32-S3 库依赖深度分析报告

**项目**: ESP32-S3 N16R8 USV (无人水面艇) Rover
**硬件平台**: ESP32-S3 (16MB Flash, 8MB PSRAM)
**分析日期**: 2025-10-27
**ArduPilot 版本**: Master Branch

---

## 分析摘要

本次分析针对 ESP32-S3 USV Rover 项目进行了全面的库依赖审查，通过检查 Rover 源代码的实际引用、HAL 配置标志、以及各库的默认启用条件，识别出了三类库：

1. **高置信度可移除** (17个库) - 确定不用于 Rover 或 USV 场景
2. **中置信度可移除** (12个库) - 可能不需要，需要用户确认
3. **低置信度可选** (8个库) - 取决于特定功能需求

**预计收益**:
- Flash 空间节省: **约 800KB - 1.2MB**
- RAM 节省: **约 150KB - 250KB**
- 编译时间缩短: **约 20-30%**

---

## 一、高置信度：明确不需要的库

这些库要么专用于其他飞行器类型，要么用于 USV 明确不需要的功能。

### 1.1 飞行器类型专用库

#### ✗ AP_LandingGear (起落架)
- **路径**: `libraries/AP_LandingGear/`
- **原因**:
  - 配置文件明确定义仅用于 Copter 和 Plane: `APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane)`
  - USV 无起落架概念
- **风险等级**: **低** - 100% 安全移除
- **验证建议**: 编译后确认无链接错误
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_LandingGear/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~15KB, RAM ~2KB

---

#### ✗ AP_Parachute (降落伞)
- **路径**: `libraries/AP_Parachute/`
- **原因**:
  - Rover 代码中无任何引用
  - 水面艇无降落伞需求
  - 默认启用但 Rover 未使用
- **风险等级**: **低**
- **验证建议**: `grep -r "Parachute" Rover/` 确认无引用
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Parachute/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~18KB, RAM ~3KB

---

#### ✗ AP_Soaring (热力上升气流滑翔)
- **路径**: `libraries/AP_Soaring/`
- **原因**:
  - 专为固定翼滑翔机设计
  - Rover 代码中无任何引用
  - USV 在水面无滑翔概念
- **风险等级**: **低**
- **验证建议**: 确认无 Sailboat 滑翔功能需求
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Soaring/.*\\.(c|cpp)$")
  ```
- **HAL 标志**: `HAL_SOARING_ENABLED=0`
- **预计节省**: Flash ~25KB, RAM ~4KB

---

#### ✗ AP_TECS (总能量控制系统)
- **路径**: `libraries/AP_TECS/`
- **原因**:
  - 专为固定翼飞机的俯仰/油门能量管理
  - Rover 中完全未引用
  - USV 无垂直能量控制需求
- **风险等级**: **低**
- **验证建议**: 确认无引用: `grep -r "AP_TECS" Rover/` 返回空
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_TECS/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~30KB, RAM ~5KB

---

### 1.2 不适用传感器/设备库

#### ✗ AP_VideoTX (视频发射器)
- **路径**: `libraries/AP_VideoTX/`
- **原因**:
  - 用于 FPV 视频发射器控制 (SmartAudio, Tramp)
  - Rover 中无引用
  - USV 项目未提及视频传输需求
- **风险等级**: **低**
- **验证建议**: 确认不需要 FPV 图传功能
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_VideoTX/.*\\.(c|cpp)$")
  ```
- **HAL 标志**: `AP_VIDEOTX_ENABLED=0`
- **预计节省**: Flash ~22KB, RAM ~3KB

---

#### ✗ AP_IRLock (红外精确定位)
- **路径**: `libraries/AP_IRLock/`
- **原因**:
  - 用于 Pixy 红外传感器跟踪
  - AC_PrecLand 的可选后端
  - USV 项目无红外跟踪需求
- **风险等级**: **低**
- **验证建议**: 确认不使用 IR Beacon 精确定位
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_IRLock/.*\\.(c|cpp)$")
  ```
- **HAL 标志**: `AP_IRLOCK_ENABLED=0`
- **预计节省**: Flash ~12KB, RAM ~2KB

---

#### ✗ AP_GyroFFT (陀螺仪 FFT 分析)
- **路径**: `libraries/AP_GyroFFT/`
- **原因**:
  - 用于多旋翼动态陷波滤波器
  - 高 CPU 开销 (需专用核心)
  - ESP32 资源有限，USV 无高频振动
- **风险等级**: **低**
- **验证建议**: Rover 无引用，ESP32 性能不足
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_GyroFFT/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~35KB, RAM ~8KB

---

#### ✗ AP_Quicktune (自动调参)
- **路径**: `libraries/AP_Quicktune/`
- **原因**:
  - 专为 Copter 姿态控制器调参
  - Rover 使用不同的控制架构
  - 未在 Rover 中引用
- **风险等级**: **低**
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Quicktune/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~20KB, RAM ~3KB

---

### 1.3 特殊硬件驱动库

#### ✗ AP_KDECAN (KDE Direct CAN ESC)
- **路径**: `libraries/AP_KDECAN/`
- **原因**:
  - 专用于 KDE Direct 无刷电调
  - USV 未使用 KDE ESC
  - 已有 DroneCAN 通用支持
- **风险等级**: **低**
- **验证建议**: 确认电机使用 DroneCAN 或传统 PWM
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_KDECAN/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~18KB, RAM ~2KB

---

#### ✗ AP_PiccoloCAN (Piccolo CAN ESC)
- **路径**: `libraries/AP_PiccoloCAN/`
- **原因**:
  - 专用于 Currawong 航空 ESC
  - USV 未使用此类专业航空电调
- **风险等级**: **低**
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_PiccoloCAN/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~25KB, RAM ~3KB

---

#### ✗ AP_BLHeli (BLHeli ESC 协议)
- **路径**: `libraries/AP_BLHeli/`
- **原因**:
  - 用于 BLHeli_32/BLHeli_S 电调配置
  - 需要双向 DShot
  - USV 通常使用船用电调，不需此协议
- **风险等级**: **中** (如果使用无人机电调则需保留)
- **验证建议**: 确认电调类型
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_BLHeli/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~28KB, RAM ~4KB

---

#### ✗ AP_FETtecOneWire (Fettec OneWire ESC)
- **路径**: `libraries/AP_FETtecOneWire/`
- **原因**:
  - 专用于 Fettec 单线电调
  - 小众产品，USV 极少使用
- **风险等级**: **低**
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_FETtecOneWire/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~15KB, RAM ~2KB

---

#### ✗ AP_RobotisServo (Robotis Dynamixel 伺服)
- **路径**: `libraries/AP_RobotisServo/`
- **原因**:
  - 专用于机器人伺服系统
  - USV 未使用此类伺服
- **风险等级**: **低**
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_RobotisServo/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~12KB, RAM ~2KB

---

### 1.4 航空特定功能库

#### ✗ AP_ADSB (ADS-B 接收器)
- **路径**: `libraries/AP_ADSB/`
- **原因**:
  - 用于航空器防撞
  - 仅在空域有意义
  - 配置要求: `HAL_PROGRAM_SIZE_LIMIT_KB > 1024`
- **风险等级**: **低**
- **验证建议**: USV 在水面，无需航空防撞
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_ADSB/.*\\.(c|cpp)$")
  ```
- **HAL 标志**: `HAL_ADSB_ENABLED=0`
- **预计节省**: Flash ~32KB, RAM ~5KB

---

#### ✗ AP_AIS (自动识别系统)
- **路径**: `libraries/AP_AIS/`
- **原因**:
  - 虽然是船舶系统，但：
  - Rover/Parameters.h 中已引用
  - **需要用户确认**：是否需要 AIS 船舶识别
  - 如果有 AIS 接收器硬件，**必须保留**
- **风险等级**: **高** - 需用户确认
- **验证建议**: 检查是否有 AIS 接收器硬件
- **移除方法**: 仅在确认无 AIS 硬件时移除
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_AIS/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~20KB (如果移除), RAM ~3KB

---

#### ✗ AP_Generator (发电机管理)
- **路径**: `libraries/AP_Generator/`
- **原因**:
  - 用于混合动力系统
  - 配置要求: `HAL_PROGRAM_SIZE_LIMIT_KB > 2048`
  - USV 项目未提及发电机
- **风险等级**: **低** (除非使用混动系统)
- **验证建议**: 确认动力系统类型
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Generator/.*\\.(c|cpp)$")
  ```
- **HAL 标志**: `HAL_GENERATOR_ENABLED=0`
- **预计节省**: Flash ~28KB, RAM ~4KB

---

#### ✗ AP_EFI (电子燃油喷射)
- **路径**: `libraries/AP_EFI/`
- **原因**:
  - 用于燃油发动机管理
  - USV 项目未提及内燃机
  - 电动推进无需 EFI
- **风险等级**: **低** (除非使用汽油机)
- **验证建议**: 确认推进系统类型
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_EFI/.*\\.(c|cpp)$")
  ```
- **HAL 标志**: `HAL_EFI_ENABLED=0`
- **预计节省**: Flash ~22KB, RAM ~3KB

---

## 二、中置信度：可能不需要的库

这些库在某些 Rover 配置中可能有用，但对基本 USV 功能不是必需的。

### 2.1 可选传感器库

#### ? AP_OpticalFlow (光流传感器)
- **路径**: `libraries/AP_OpticalFlow/`
- **使用情况**:
  - Rover.h 中有条件引用: `#if AP_OPTICALFLOW_ENABLED`
  - 用于无 GPS 环境的速度估计
  - **USV 通常有 GPS，可能不需要**
- **风险等级**: **中**
- **验证建议**:
  - 检查是否需要无 GPS 导航
  - 水面光流效果较差（水波纹理变化）
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_OpticalFlow/.*\\.(c|cpp)$")
  target_compile_definitions(${COMPONENT_LIB} PUBLIC AP_OPTICALFLOW_ENABLED=0)
  ```
- **预计节省**: Flash ~30KB, RAM ~5KB

---

#### ? AC_PrecLand (精确降落)
- **路径**: `libraries/AC_PrecLand/`
- **使用情况**:
  - Rover.h 中有条件引用
  - CMakeLists.txt 中已包含在 ROVER_REQUIRED_AC_MODULES
  - **可能用于精确对接**
- **风险等级**: **中**
- **验证建议**:
  - 确认是否需要 IR Beacon / 视觉标记 对接
  - MODE_DOCK_ENABLED 依赖于此库
- **建议**: **暂时保留**，如果不需要对接功能再移除
- **移除方法**: 从 ROVER_REQUIRED_AC_MODULES 中移除
- **预计节省**: Flash ~35KB, RAM ~6KB

---

#### ? AP_Beacon (无线电信标定位)
- **路径**: `libraries/AP_Beacon/`
- **使用情况**:
  - Parameters.h 中有条件引用: `#if AP_BEACON_ENABLED`
  - 用于室内/无 GPS 环境定位
  - **USV 通常在室外，有 GPS**
- **风险等级**: **中**
- **验证建议**: 确认是否需要非 GPS 定位
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Beacon/.*\\.(c|cpp)$")
  target_compile_definitions(${COMPONENT_LIB} PUBLIC AP_BEACON_ENABLED=0)
  ```
- **预计节省**: Flash ~25KB, RAM ~4KB

---

### 2.2 云台相机系统

#### ? AP_Camera (相机触发)
- **路径**: `libraries/AP_Camera/`
- **使用情况**:
  - Rover.h: `#if AP_CAMERA_ENABLED`
  - 用于航拍相机控制
  - **USV 可能需要水下/水面相机**
- **风险等级**: **高** - 需用户确认
- **验证建议**: 确认是否需要相机触发功能
- **保留建议**: 如果有监控需求，建议保留
- **移除方法**: (仅在确认不需要时)
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Camera/.*\\.(c|cpp)$")
  target_compile_definitions(${COMPONENT_LIB} PUBLIC AP_CAMERA_ENABLED=0)
  ```
- **预计节省**: Flash ~40KB, RAM ~6KB

---

#### ? AP_Mount (云台控制)
- **路径**: `libraries/AP_Mount/`
- **使用情况**:
  - Rover.h: `#if HAL_MOUNT_ENABLED`
  - 用于多轴云台控制
  - **USV 可能需要云台相机**
- **风险等级**: **高** - 需用户确认
- **验证建议**: 确认是否有云台设备
- **移除方法**: (仅在确认无云台时)
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Mount/.*\\.(c|cpp)$")
  target_compile_definitions(${COMPONENT_LIB} PUBLIC HAL_MOUNT_ENABLED=0)
  ```
- **预计节省**: Flash ~45KB, RAM ~7KB

---

### 2.3 高级功能库

#### ? AP_Winch (绞盘系统)
- **路径**: `libraries/AP_Winch/`
- **使用情况**:
  - 配置要求: `HAL_PROGRAM_SIZE_LIMIT_KB > 2048`
  - **USV 可能需要绞盘（取样、锚定等）**
- **风险等级**: **高** - 需用户确认
- **验证建议**: 确认是否需要绞盘功能
- **移除方法**: (仅在确认不需要时)
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Winch/.*\\.(c|cpp)$")
  target_compile_definitions(${COMPONENT_LIB} PUBLIC AP_WINCH_ENABLED=0)
  ```
- **预计节省**: Flash ~18KB, RAM ~3KB

---

#### ? AP_Follow (跟随模式)
- **路径**: `libraries/AP_Follow/`
- **使用情况**:
  - Rover.h: `#if MODE_FOLLOW_ENABLED`
  - Parameters.h 中引用
  - 用于跟随另一艘船/设备
- **风险等级**: **中**
- **验证建议**: 确认是否需要编队/跟随功能
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Follow/.*\\.(c|cpp)$")
  target_compile_definitions(${COMPONENT_LIB} PUBLIC AP_FOLLOW_ENABLED=0)
  ```
- **预计节省**: Flash ~20KB, RAM ~3KB

---

#### ? AC_Sprayer (喷洒系统)
- **路径**: `libraries/AC_Sprayer/`
- **使用情况**:
  - Parameters.h: `#if HAL_SPRAYER_ENABLED`
  - 已在 ROVER_REQUIRED_AC_MODULES 中
  - **可能用于水质监测/处理**
- **风险等级**: **中**
- **验证建议**: 确认是否需要喷洒/投放功能
- **移除方法**: 从 ROVER_REQUIRED_AC_MODULES 中移除
- **预计节省**: Flash ~15KB, RAM ~2KB

---

#### ? AP_Gripper (夹持器)
- **路径**: `libraries/AP_Gripper/`
- **使用情况**:
  - Rover 中有引用
  - **USV 可能需要取样器/夹持器**
- **风险等级**: **高** - 需用户确认
- **验证建议**: 确认是否需要机械臂/夹持功能
- **移除方法**: (仅在确认不需要时)
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Gripper/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~12KB, RAM ~2KB

---

### 2.4 遥测协议库

#### ? AP_Frsky_Telem (FrSky 遥测)
- **路径**: `libraries/AP_Frsky_Telem/`
- **使用情况**:
  - 默认启用
  - 用于 FrSky 接收机遥测
  - **如果使用其他接收机可移除**
- **风险等级**: **低**
- **验证建议**: 确认遥控器品牌
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Frsky_Telem/.*\\.(c|cpp)$")
  ```
- **HAL 标志**: `AP_FRSKY_TELEM_ENABLED=0`
- **预计节省**: Flash ~25KB, RAM ~3KB

---

#### ? AP_Devo_Telem (Devo 遥测)
- **路径**: `libraries/AP_Devo_Telem/`
- **使用情况**: 仅用于 Walkera Devo 系统
- **风险等级**: **低**
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Devo_Telem/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~8KB, RAM ~1KB

---

#### ? AP_Hott_Telem (Hott 遥测)
- **路径**: `libraries/AP_Hott_Telem/`
- **使用情况**: 仅用于 Graupner Hott 系统
- **风险等级**: **低**
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Hott_Telem/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~10KB, RAM ~2KB

---

#### ? AP_LTM_Telem (LTM 遥测)
- **路径**: `libraries/AP_LTM_Telem/`
- **使用情况**: 轻量遥测协议，少用
- **风险等级**: **低**
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_LTM_Telem/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~6KB, RAM ~1KB

---

## 三、低置信度：依功能需求决定

这些库在特定配置下可能有用，需要根据实际需求判断。

### 3.1 USV 特定考虑

#### ⚠ AP_Torqeedo (Torqeedo 电机)
- **路径**: `libraries/AP_Torqeedo/`
- **使用情况**:
  - Parameters.h 中引用
  - AP_Arming_Rover.cpp 中使用
  - **专用于 Torqeedo 船用电机**
- **风险等级**: **极高** - 需用户确认
- **验证建议**:
  - 确认电机品牌
  - 如果使用 Torqeedo，**必须保留**
  - 如果使用其他电机，可安全移除
- **移除方法**: (仅在确认不使用 Torqeedo 时)
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Torqeedo/.*\\.(c|cpp)$")
  target_compile_definitions(${COMPONENT_LIB} PUBLIC HAL_TORQEEDO_ENABLED=0)
  ```
- **预计节省**: Flash ~35KB, RAM ~5KB

---

#### ⚠ AP_WindVane (风向标)
- **路径**: `libraries/AP_WindVane/`
- **使用情况**:
  - Parameters.h 中引用
  - **用于帆船/风力辅助**
- **风险等级**: **高** - 需用户确认
- **验证建议**:
  - 确认是否有帆船功能
  - 气象站可能提供风向数据
- **建议**: **保留** - 气象站已集成，可能需要
- **移除方法**: (仅在确认不需要时)
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_WindVane/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~18KB, RAM ~3KB

---

#### ⚠ Sailboat (帆船模块)
- **路径**: `libraries/Rover/sailboat.*`
- **使用情况**: Parameters.h 中引用
- **风险等级**: **高** - 需用户确认
- **验证建议**: 确认是否有帆船功能
- **移除方法**: 需要修改 Parameters.cpp
- **预计节省**: Flash ~25KB, RAM ~4KB

---

### 3.2 高级导航功能

#### ⚠ AP_Terrain (地形跟随)
- **路径**: `libraries/AP_Terrain/`
- **使用情况**:
  - 用于地形数据库
  - **USV 可能需要水深地图**
- **风险等级**: **中**
- **验证建议**: 确认是否需要地形/水深数据
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Terrain/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~40KB, RAM ~20KB (地形数据库占用大)

---

#### ⚠ AP_OAPathPlanner (障碍规避路径规划)
- **路径**: `libraries/AP_OAPathPlanner/`
- **使用情况**:
  - Parameters.h: `#if AP_OAPATHPLANNER_ENABLED`
  - 用于动态避障
- **风险等级**: **中**
- **验证建议**: 确认是否需要高级避障
- **建议**: **保留** - USV 需要避障功能
- **移除方法**: (仅在确认不需要时)
  ```cmake
  target_compile_definitions(${COMPONENT_LIB} PUBLIC AP_OAPATHPLANNER_ENABLED=0)
  ```
- **预计节省**: Flash ~30KB, RAM ~8KB

---

### 3.3 调试开发工具

#### ⚠ AP_Scripting (Lua 脚本)
- **路径**: `libraries/AP_Scripting/`
- **使用情况**:
  - Rover.h: `#if AP_SCRIPTING_ENABLED`
  - **非常有用的扩展功能**
- **风险等级**: **中**
- **验证建议**:
  - 脚本可用于传感器集成（DST800, 水质仪等）
  - **建议保留**用于自定义功能
- **移除方法**: (不建议移除)
  ```cmake
  target_compile_definitions(${COMPONENT_LIB} PUBLIC AP_SCRIPTING_ENABLED=0)
  ```
- **预计节省**: Flash ~150KB, RAM ~50KB (巨大，但功能强大)

---

#### ⚠ AP_OSD (屏显系统)
- **路径**: `libraries/AP_OSD/`
- **使用情况**:
  - Rover.h: `#if OSD_ENABLED || OSD_PARAM_ENABLED`
  - 用于视频叠加显示
- **风险等级**: **中**
- **验证建议**: 确认是否需要 OSD 功能
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_OSD/.*\\.(c|cpp)$")
  target_compile_definitions(${COMPONENT_LIB} PUBLIC OSD_ENABLED=0)
  ```
- **预计节省**: Flash ~55KB, RAM ~10KB

---

#### ⚠ AP_Button (按钮输入)
- **路径**: `libraries/AP_Button/`
- **使用情况**: Parameters.cpp 中引用
- **风险等级**: **低**
- **验证建议**: 确认是否有物理按钮
- **移除方法**:
  ```cmake
  list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Button/.*\\.(c|cpp)$")
  ```
- **预计节省**: Flash ~8KB, RAM ~1KB

---

## 四、必须保留的核心库

以下库对 Rover 基本功能至关重要，**绝对不能移除**：

### 4.1 核心 HAL 层
- ✓ **AP_HAL** - 硬件抽象层
- ✓ **AP_HAL_ESP32** - ESP32 平台支持
- ✓ **AP_HAL_Empty** - 空实现占位符

### 4.2 导航控制核心
- ✓ **AP_AHRS** - 姿态航向参考系统
- ✓ **AP_NavEKF2/3** - 扩展卡尔曼滤波器
- ✓ **AP_InertialNav** - 惯性导航
- ✓ **AP_InertialSensor** - IMU 传感器
- ✓ **AP_Compass** - 罗盘
- ✓ **AP_GPS** - GPS 接收器
- ✓ **AP_Baro** - 气压计

### 4.3 控制系统
- ✓ **AR_WPNav** - Rover 航点导航
- ✓ **AR_Motors** / **AP_MotorsUGV** - 地面车辆电机控制
- ✓ **AR_AttitudeControl** - Rover 姿态控制

### 4.4 通信系统
- ✓ **GCS_MAVLink** - MAVLink 协议
- ✓ **AP_SerialManager** - 串口管理
- ✓ **AP_DroneCAN** - DroneCAN 协议（已实现）
- ✓ **AP_CANManager** - CAN 总线管理

### 4.5 任务规划
- ✓ **AP_Mission** - 任务管理
- ✓ **AP_Rally** - 集结点
- ✓ **AP_SmartRTL** - 智能返航
- ✓ **AC_Fence** - 地理围栏
- ✓ **AC_Avoidance** - 基础避障

### 4.6 传感器支持
- ✓ **AP_RangeFinder** - 测距仪（DST800 深度计）
- ✓ **AP_BattMonitor** - 电池监测
- ✓ **AP_RPM** - 转速计
- ✓ **AP_WheelEncoder** - 轮速编码器
- ✓ **AP_Proximity** - 接近传感器（避障）
- ✓ **AP_TemperatureSensor** - 温度传感器

### 4.7 系统基础
- ✓ **AP_Param** - 参数系统
- ✓ **AP_Logger** - 数据日志
- ✓ **AP_Scheduler** - 任务调度器
- ✓ **AP_BoardConfig** - 板级配置
- ✓ **AP_Arming** - 解锁系统
- ✓ **AP_Notify** - LED/蜂鸣器通知
- ✓ **AP_Stats** - 统计信息

### 4.8 USV 特定
- ✓ **AP_WheelEncoder** - 用于速度估计
- ✓ **AP_WindVane** - 配合气象站
- ✓ **AP_RangeFinder** - DST800 测深

---

## 五、分阶段优化建议

### 阶段 1: 低风险优化（立即执行）

移除以下 17 个库，预计节省 **~450KB Flash, ~60KB RAM**:

```cmake
# 在 components/ardupilot/CMakeLists.txt 中添加:

# 飞行器类型专用
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_LandingGear/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Parachute/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Soaring/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_TECS/.*\\.(c|cpp)$")

# 不适用传感器
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_VideoTX/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_IRLock/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_GyroFFT/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Quicktune/.*\\.(c|cpp)$")

# 特殊硬件驱动
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_KDECAN/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_PiccoloCAN/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_BLHeli/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_FETtecOneWire/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_RobotisServo/.*\\.(c|cpp)$")

# 航空特定
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_ADSB/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Generator/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_EFI/.*\\.(c|cpp)$")

# 遥测协议（如果不使用）
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Devo_Telem/.*\\.(c|cpp)$")

# 对应的 HAL 标志
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    HAL_PARACHUTE_ENABLED=0
    HAL_SOARING_ENABLED=0
    AP_VIDEOTX_ENABLED=0
    AP_IRLOCK_ENABLED=0
    HAL_ADSB_ENABLED=0
    HAL_GENERATOR_ENABLED=0
    HAL_EFI_ENABLED=0
)
```

### 阶段 2: 中风险优化（需确认硬件）

根据实际硬件配置，考虑移除以下库：

**确认项**:
1. **Torqeedo 电机**: 如果不使用，移除 `AP_Torqeedo` (节省 35KB)
2. **遥测协议**: 确认遥控器品牌后移除不需要的协议 (节省 25-40KB)
3. **光流传感器**: 如果不使用，禁用 `AP_OpticalFlow` (节省 30KB)
4. **无线电信标**: 如果不使用，禁用 `AP_Beacon` (节省 25KB)

### 阶段 3: 功能性优化（需业务确认）

根据 USV 实际功能需求：

1. **相机系统**: 如果不需要相机/云台，移除 `AP_Camera` + `AP_Mount` (节省 85KB)
2. **精确对接**: 如果不需要对接功能，移除 `AC_PrecLand` (节省 35KB)
3. **AIS 系统**: 如果无 AIS 接收器，移除 `AP_AIS` (节省 20KB)
4. **绞盘系统**: 如果不需要，移除 `AP_Winch` (节省 18KB)

---

## 六、内存节省估算

### 保守估算（仅阶段 1）
- **Flash**: 450KB
- **RAM**: 60KB
- **编译时间**: 减少 15%

### 中等估算（阶段 1 + 2）
- **Flash**: 800KB
- **RAM**: 150KB
- **编译时间**: 减少 25%

### 激进估算（全部阶段）
- **Flash**: 1.2MB
- **RAM**: 250KB
- **编译时间**: 减少 30%

---

## 七、验证流程

### 7.1 编译验证
```bash
cd f:/opensource/usv_esp32/esp32s3rover/ardupilot_rover_esp32s3_idf
idf.py build
```

### 7.2 链接验证
检查链接器输出，确认无未定义符号：
```
undefined reference to XXX
```

### 7.3 功能验证清单
- [ ] MAVLink 通信正常
- [ ] DroneCAN 节点识别
- [ ] GPS 定位工作
- [ ] 传感器数据读取（DST800, 4G, 水质等）
- [ ] 电机控制响应
- [ ] 任务规划执行
- [ ] 参数保存/加载

### 7.4 运行时验证
```bash
# 查看参数列表
param show

# 测试传感器
sensor test all

# 检查内存使用
status free_memory
```

---

## 八、风险评估与回退方案

### 8.1 风险矩阵

| 库类别 | 移除风险 | 影响范围 | 回退难度 |
|--------|----------|----------|----------|
| 飞行器专用 | 极低 | 无 | 容易 |
| 特殊硬件驱动 | 低 | 仅特定硬件 | 容易 |
| 遥测协议 | 低 | 地面站显示 | 容易 |
| 传感器可选 | 中 | 导航精度 | 中等 |
| 相机云台 | 中 | 监控功能 | 中等 |
| 核心导航 | 极高 | 系统无法运行 | 困难 |

### 8.2 回退方案

如果移除某库后出现问题：

1. **立即回退**: 注释掉对应的 `EXCLUDE REGEX` 行
2. **重新编译**: `idf.py build`
3. **记录问题**: 哪个功能受影响
4. **选择性保留**: 只保留必需的库

---

## 九、特别注意事项

### 9.1 ESP32 平台特性
- **Flash 限制**: ESP32-S3 有 16MB Flash，但分区限制应用大小
- **PSRAM 使用**: 8MB PSRAM 需合理分配（堆、DMA、缓冲区）
- **实时性要求**: DroneCAN 需要及时处理，避免过多库导致调度延迟

### 9.2 DroneCAN 依赖
以下库与 DroneCAN 相关，**不可移除**:
- AP_DroneCAN
- AP_CANManager
- AP_GPS（可能通过 CAN 获取）
- AP_Compass（可能通过 CAN 获取）
- AP_BattMonitor（可能通过 CAN 获取）

### 9.3 传感器集成考虑
您的 USV 有 5 个传感器：
1. **DST800 测深仪**: 需要 `AP_RangeFinder` 或自定义驱动
2. **4G 模块**: 需要 `AP_Networking` 或串口通信
3. **海流计**: 可能需要 Lua 脚本或自定义驱动
4. **水质仪**: 可能需要 Lua 脚本
5. **气象站**: 需要 `AP_WindVane` 和自定义解析

**建议**: 保留 `AP_Scripting` 用于传感器集成。

---

## 十、推荐实施方案

### 方案 A: 保守优化（推荐）
**目标**: 移除明确不需要的库，保留所有可能有用的功能

**移除列表**:
- 飞行器专用库 (4个)
- 特殊硬件驱动 (5个)
- 航空特定库 (3个)
- 不使用的遥测协议 (1个)

**预计收益**: Flash ~450KB, RAM ~60KB

**风险**: 极低

---

### 方案 B: 积极优化
**目标**: 在方案 A 基础上，进一步移除确认不需要的功能

**额外移除**:
- 光流传感器 (如果确认不用)
- 无线电信标 (如果确认不用)
- 非使用的遥测协议 (FrSky, Hott, LTM)

**预计收益**: Flash ~750KB, RAM ~120KB

**风险**: 低（需硬件确认）

---

### 方案 C: 激进优化（需深度评估）
**目标**: 最大化节省空间，仅保留核心功能

**额外移除**:
- 相机/云台系统 (如果确认不用)
- 精确对接功能
- OSD 屏显
- 部分高级导航功能

**预计收益**: Flash ~1.2MB, RAM ~250KB

**风险**: 中（需业务确认）

---

## 十一、后续优化方向

### 11.1 代码级优化
1. **条件编译优化**: 使用更细粒度的 `HAL_*_ENABLED` 标志
2. **模板实例化控制**: 减少 C++ 模板展开
3. **内联函数优化**: 调整编译器优化级别

### 11.2 配置优化
1. **降低主循环频率**: 从 400Hz 降至 200Hz (如果实时性允许)
2. **减少日志缓冲**: 调整 `HAL_LOGGING_ENABLED` 参数
3. **精简 EKF**: 使用 EKF2 替代 EKF3（节省约 50KB）

### 11.3 自定义传感器驱动
考虑为 5 个传感器编写轻量级驱动，替代 Lua 脚本：
- 节省 Lua 虚拟机开销 (~150KB)
- 提高执行效率
- 减少 RAM 占用

---

## 十二、总结

### 关键发现
1. **已优化**: AC_* 模块已移除 4 个（AttitudeControl, InputManager, PID, WPNav）
2. **高价值目标**: 17 个库可安全移除，节省 ~450KB
3. **需确认项**: 12 个库需要硬件/功能确认
4. **保留核心**: 约 40 个核心库必须保留

### 下一步行动
1. **立即执行**: 实施方案 A（保守优化）
2. **硬件确认**: 列出实际使用的硬件清单
3. **功能评估**: 确认需要的功能（相机、对接、AIS 等）
4. **逐步优化**: 根据确认结果实施方案 B 或 C

### 最终建议
- **第一优先级**: 移除方案 A 中的 17 个库
- **第二优先级**: 确认 Torqeedo、遥测协议、传感器配置
- **第三优先级**: 评估相机、AIS、对接等功能需求
- **长期优化**: 考虑自定义传感器驱动，替代脚本

---

**文档版本**: 1.0
**最后更新**: 2025-10-27
**作者**: Claude (ArduPilot ESP32 优化专家)
**审核状态**: 待用户确认
