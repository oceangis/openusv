# ArduPilot Rover ESP32 库精简方案

## 当前问题
当前编译包含了所有ArduPilot库,导致:
- 编译时间过长
- 固件体积过大
- 包含了大量Rover不需要的功能

## Rover 实际需要的核心库

### 必需库(Rover核心功能)
1. **基础HAL和通用库**
   - AP_HAL (硬件抽象层)
   - AP_HAL_ESP32 (ESP32特定HAL)
   - AP_Common (通用工具)
   - AP_Math (数学库)
   - AP_Param (参数系统)
   - AP_BoardConfig (板级配置)
   - AP_Vehicle (车辆基类)
   - StorageManager (存储管理)

2. **传感器和导航**
   - AP_AHRS (姿态航向参考系统)
   - AP_InertialSensor (惯性传感器 - IMU)
   - AP_Compass (罗盘/磁力计)
   - AP_GPS (GPS)
   - AP_Baro (气压计 - 可选)
   - AP_RangeFinder (测距仪)
   - AP_WheelEncoder (轮速编码器)
   - AP_NavEKF2/3 (扩展卡尔曼滤波器)

3. **控制系统**
   - AR_Motors (Rover电机控制)
   - AR_WPNav (Rover航点导航)
   - APM_Control (姿态控制)
   - AC_PID (PID控制器)

4. **通信和遥测**
   - GCS_MAVLink (MAVLink通信)
   - RC_Channel (遥控输入)
   - AP_RCProtocol (RC协议解析)
   - AP_SerialManager (串口管理)

5. **任务和导航**
   - AP_Mission (任务管理)
   - AP_Rally (集结点)
   - AP_SmartRTL (智能返航)
   - AC_Fence (电子围栏)
   - AC_Avoidance (避障)

6. **监控和日志**
   - AP_Logger (数据记录)
   - AP_BattMonitor (电池监控)
   - AP_Arming (解锁检查)
   - AP_Scheduler (任务调度)
   - AP_Notify (通知系统)

7. **其他实用功能**
   - AP_Filesystem (文件系统)
   - AP_RTC (实时时钟)
   - Filter (滤波器)

### 可选库(根据配置启用)
- AP_Camera (相机触发)
- AP_Mount (云台控制)
- AP_OSD (屏显)
- AP_Scripting (Lua脚本)
- AP_Proximity (接近传感器)
- AP_Beacon (信标)
- AP_OpticalFlow (光流)
- AP_RPM (转速计)
- AP_WindVane (风向标 - 帆船)
- AP_Torqeedo (Torqeedo电机)
- AP_Follow (跟随模式)
- AP_ExternalControl (外部控制)
- AC_Sprayer (喷雾器)
- AP_Gripper (夹爪)
- AC_PrecLand (精确着陆)

## 明确不需要的库(飞行器专用)

### 多旋翼/直升机专用
- AC_AttitudeControl (多旋翼姿态控制)
- AC_WPNav (多旋翼航点导航)
- AC_AutoTune (自动调参)
- AC_Autorotation (自旋下降)
- AC_CustomControl (自定义控制)
- AC_InputManager (输入管理器)
- AP_LandingGear (起落架)
- AP_Winch (绞盘)
- AP_Parachute (降落伞)

### 固定翼专用
- AP_Airspeed (空速传感器及所有后端)
- AP_TECS (总能量控制系统)
- AP_L1_Control (L1导航控制器)
- AP_Landing (着陆控制)
- AP_Soaring (滑翔)
- AP_ADSB (ADS-B - 主要用于飞行器)
- AP_Avoidance (ADSB避障 - 固定翼)

### 潜水器专用
- AP_LeakDetector (漏水检测)
- AP_SurfaceDistance (水面距离)
- AP_TemperatureSensor (温度传感器)

### 特殊应用
- AP_ICEngine (内燃机)
- AP_EFI (电子燃油喷射)
- AP_Generator (发电机)
- AP_IOMCU (IO协处理器 - Pixhawk特定)
- AP_BLHeli (BLHeli电调配置)
- AP_VideoTX (图传)
- AP_FETtecOneWire (FETtec电调)
- AP_KDECAN (KDE CAN电调)
- AP_PiccoloCAN (Piccolo CAN)
- AP_RobotisServo (Robotis舵机)
- AP_SBusOut (SBus输出)
- AP_Volz_Protocol (Volz协议)

### 开发/测试工具
- AP_DAL (数据抽象层 - 用于replay)
- AP_CheckFirmware (固件检查 - 加密签名)
- AP_Devo_Telem (Devo遥测)
- AP_Frsky_Telem (FrSky遥测 - 可选)
- AP_Hott_Telem (HoTT遥测 - 可选)
- AP_LTM_Telem (LTM遥测 - 可选)
- AP_MSP (MSP协议 - 可选)

### 网络相关(ESP32可能需要部分)
- AP_Networking (网络功能)
- AP_DDS (DDS支持)
- AP_ONVIF (ONVIF协议)

### 其他不常用
- AP_Button (按钮)
- AP_DAC (数模转换)
- AP_Declination (磁偏角)
- AP_ESC_Telem (ESC遥测 - 可选)
- AP_GSOF (GSOF协议)
- AP_OLC (开放位置代码)
- AP_OpenDroneID (Remote ID)
- AP_RAMTRON (RAMTRON存储)
- AP_VisualOdom (视觉里程计)
- AP_AIS (船舶自动识别系统 - USV可能需要)
- AP_CSVReader (CSV读取器)
- AP_JSON (JSON解析)
- AP_Menu (菜单系统)
- AP_Module (模块系统)
- AP_OADatabase (障碍物数据库)
- AP_Stats (统计)
- AP_TempCalibration (温度校准)
- AP_Tuning (调参)

## 精简建议

### 第一阶段:删除明确不需要的库
移除以下库可以显著减少编译时间和固件大小:
1. 所有多旋翼专用库(AC_AttitudeControl, AC_WPNav等)
2. 所有固定翼专用库(AP_Airspeed, AP_TECS等)
3. 特殊硬件库(AP_ICEngine, AP_EFI, AP_BLHeli等)
4. 不使用的通信协议库(AP_Devo_Telem, AP_Hott_Telem等)

### 第二阶段:条件编译可选功能
通过配置宏控制:
- AP_Camera (如不使用相机)
- AP_Mount (如不使用云台)
- AP_OSD (如不使用OSD)
- AP_Scripting (如不使用脚本)
- 各种遥测协议

### 第三阶段:优化传感器后端
只保留实际使用的传感器驱动:
- GPS: 只保留使用的GPS类型(uBlox/NMEA等)
- Compass: 只保留使用的罗盘芯片
- Baro: 只保留使用的气压计芯片
- IMU: 只保留使用的IMU芯片(如ICM20948)

## 实施方法

### 方法1: 修改CMakeLists.txt
在 `components/ardupilot/CMakeLists.txt` 中注释掉不需要的库源文件

### 方法2: 使用配置宏
在编译时定义宏来禁用功能:
```cpp
#define HAL_MOUNT_ENABLED 0
#define HAL_SPRAYER_ENABLED 0
#define HAL_ADSB_ENABLED 0
// 等等
```

### 方法3: 创建精简的hwdef
在hwdef.dat中禁用不需要的功能

## 预期效果
- 编译时间减少 40-60%
- 固件大小减少 30-50%
- 内存占用减少
- 更清晰的依赖关系
