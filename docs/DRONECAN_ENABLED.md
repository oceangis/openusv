# DroneCAN已启用 - 配置完成

## 完成时间
2025-10-29

## 已完成的配置

### 1. libcanard库已添加 ✅
```
modules/DroneCAN/libcanard/
├── canard.c
├── canard.h
├── canard_internals.h
├── canard/
│   ├── interface.h
│   ├── callbacks.h
│   ├── publisher.h
│   ├── subscriber.h
│   └── ...
└── drivers/
```

**来源**: 从GitHub克隆 `https://github.com/DroneCAN/libcanard`

### 2. esp32.h DroneCAN配置已启用 ✅

文件: `libraries/AP_HAL/board/esp32.h`

```c
#define HAL_WITH_DRONECAN 1
#define HAL_WITH_UAVCAN 1
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 1
#define HAL_NUM_CAN_IFACES 1
```

### 3. hwdef.dat CAN接口已配置 ✅

文件: `libraries/AP_HAL_ESP32/hwdef/esp32s3_icm20948/hwdef.dat`

```python
# DroneCAN / CAN on GPIO47(TX) / GPIO38(RX)
# TWAI (Two-Wire Automotive Interface) is ESP32's CAN peripheral
# Bitrate: 1Mbps (standard for DroneCAN)
ESP32_CAN  TWAI_NUM_0  GPIO_NUM_47  GPIO_NUM_38  1000000
define HAL_NUM_CAN_IFACES 1
define HAL_ENABLE_DRONECAN_DRIVERS 1
```

**硬件连接**:
- CAN_TX: GPIO47
- CAN_RX: GPIO38
- 波特率: 1Mbps (DroneCAN标准)

### 4. hwdef.h已重新生成 ✅

文件: `libraries/AP_HAL_ESP32/hwdef/hwdef.h`

生成的配置包含:
```c
#define HAL_NUM_CAN_IFACES 1
#define HAL_ENABLE_DRONECAN_DRIVERS 1
```

### 5. CANIface驱动已存在 ✅

ESP32-S3的CAN驱动文件:
- `libraries/AP_HAL_ESP32/CANIface.h`
- `libraries/AP_HAL_ESP32/CANIface.cpp`

这些文件实现了ESP32-S3的TWAI (CAN)接口。

## 完整配置概览

```
项目: ArduPilot Rover ESP32-S3 IDF版本
主控板: ESP32-S3 N16R8

硬件外设配置:
├── IMU: ICM20948 (I2C @ GPIO20/21)
├── GPS: u-blox MAX-M10S (UART1 @ GPIO17/18, 38400)
├── PWM: GPIO11-6 (6通道)
├── CAN: GPIO47(TX)/38(RX) (1Mbps) ✅ 新增
├── WiFi: ardupilot123
└── PSRAM: 8MB已启用

DroneCAN功能:
├── libcanard协议库
├── TWAI硬件接口
├── ESC遥测支持
├── 智能设备通信
└── 多节点网络
```

## 下一步操作

### 编译测试

使用build.bat脚本编译:
```batch
cd f:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf
build.bat build
```

如果编译成功，继续烧录测试。

### 硬件需求

要实际使用DroneCAN，还需要：

1. **CAN收发器** (例如SN65HVD230或TJA1050)
   ```
   ESP32-S3          CAN Transceiver
   GPIO47(TX)  -->   TXD
   GPIO38(RX)  <--   RXD
   GND         ---   GND
   3.3V        ---   VCC
                     CANH  -->  CAN总线
                     CANL  -->  CAN总线
   ```

2. **终端电阻** (120Ω)
   - CAN总线两端各需要一个120Ω电阻

3. **DroneCAN设备**
   - ESC (电调)
   - GPS/COMPASS模块
   - 其他DroneCAN智能设备

### 参数配置

通过MAVLink或Mission Planner配置:
```
CAN_P1_DRIVER = 1     # 启用第一个CAN端口的DroneCAN
CAN_D1_PROTOCOL = 1   # CAN端口1使用DroneCAN协议
CAN_P1_BITRATE = 1000000  # 1Mbps波特率
```

### 测试步骤

1. **编译固件**
   ```batch
   build.bat clean
   build.bat build
   ```

2. **烧录固件**
   ```batch
   build.bat erase
   build.bat flash
   ```

3. **连接CAN设备**
   - 连接CAN收发器到GPIO47/38
   - 连接DroneCAN设备到CAN总线

4. **监控串口输出**
   ```batch
   build.bat monitor
   ```

   查找类似以下的日志:
   ```
   CAN: initialized driver 1
   DroneCAN: starting node
   DroneCAN: discovered device...
   ```

5. **通过MAVLink验证**
   - 连接Mission Planner或QGroundControl
   - 检查CAN设备是否被识别
   - 查看ESC遥测数据（如果连接了ESC）

## 技术细节

### TWAI (Two-Wire Automotive Interface)

ESP32-S3使用TWAI作为CAN外设:
- 兼容CAN 2.0B规范
- 支持标准帧和扩展帧
- 硬件过滤器支持
- 自动重传功能
- 错误检测和恢复

### DroneCAN协议栈

```
应用层: ArduPilot Vehicle Code
         ↓
协议层: AP_DroneCAN (DroneCAN drivers)
         ↓
库层:   libcanard (DroneCAN protocol)
         ↓
HAL层:  CANIface (ESP32 TWAI wrapper)
         ↓
硬件层: ESP-IDF TWAI Driver
         ↓
物理层: ESP32-S3 TWAI Peripheral
```

### 内存占用估算

启用DroneCAN后的额外内存需求:
- Flash: ~50KB (libcanard + DroneCAN drivers)
- RAM: ~10KB (运行时缓冲区)

8MB PSRAM足以支持DroneCAN功能。

## 故障排查

### 编译错误

如果遇到libcanard相关编译错误:
```bash
# 检查libcanard是否正确克隆
ls -la modules/DroneCAN/libcanard/canard/interface.h

# 如果文件不存在，重新克隆
cd modules/DroneCAN
rm -rf libcanard
git clone https://github.com/DroneCAN/libcanard.git
```

### CAN通信问题

1. **检查硬件连接**
   - 确认CAN收发器正确连接
   - 检查终端电阻(120Ω)
   - 验证电源供电

2. **检查波特率**
   - 确保所有CAN设备使用相同波特率(1Mbps)

3. **查看日志**
   ```batch
   build.bat monitor
   ```
   查找错误信息

4. **使用CAN分析仪**
   - 建议使用CAN分析仪检测总线通信

## 相关文档

- `BUILD_GUIDE.md` - 编译和烧录指南
- `DRONECAN_IMPLEMENTATION_PLAN.md` - DroneCAN实现计划
- `DRONECAN_QUICK_START.md` - DroneCAN快速入门

## 参考资源

- DroneCAN官方: https://dronecan.github.io/
- libcanard仓库: https://github.com/DroneCAN/libcanard
- ArduPilot CAN文档: https://ardupilot.org/copter/docs/common-canbus-setup-advanced.html
- ESP32-S3 TWAI文档: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/twai.html

---

**状态**: ✅ DroneCAN配置完成，准备编译测试
**最后更新**: 2025-10-29

## CMakeLists.txt修改 (2025-10-29更新)

### 添加的配置

在 `components/ardupilot/CMakeLists.txt` 中添加了libcanard支持：

1. **libcanard源文件收集** (第20-23行):
```cmake
# Collect DroneCAN libcanard source files
file(GLOB LIBCANARD_SRCS
    "../../modules/DroneCAN/libcanard/canard.c"
)
```

2. **合并到源文件列表** (第26行):
```cmake
set(COMPONENT_SRCS ${ALL_LIBRARY_SRCS} ${ALL_ROVER_SRCS} ${LIBCANARD_SRCS})
```

3. **添加include路径** (第293-294行):
```cmake
# Add DroneCAN libcanard include paths
list(APPEND COMPONENT_ADD_INCLUDEDIRS "../../modules/DroneCAN/libcanard")
```

这些修改解决了 `fatal error: canard/interface.h: No such file or directory` 编译错误。

