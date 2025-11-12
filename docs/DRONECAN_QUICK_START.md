# ESP32-S3 DroneCAN 快速入门

**5 分钟快速启动 DroneCAN 通信**

---

## 准备工作

### 硬件清单
- ✅ ESP32-S3 开发板（16MB Flash, 8MB PSRAM）
- ✅ SN65HVD230 CAN 收发器模块
- ✅ ArduPilot 飞控（或其他 DroneCAN 设备）
- ✅ 杜邦线若干
- ✅ 120Ω 电阻 × 2（CAN 终端电阻）

### 软件环境
- ✅ ESP-IDF 5.5.1 已安装
- ✅ 本项目代码已下载

---

## Step 1: 硬件连接（2 分钟）

### 连接 SN65HVD230

```
ESP32-S3              SN65HVD230
GPIO 47 (TX)  ------>  TXD
GPIO 38 (RX)  <------  RXD
3.3V          ------>  VCC
GND           ------>  GND
```

### 连接到 CAN 总线

```
SN65HVD230            CAN Bus
CANH          ------>  CAN_H  ----------+
CANL          ------>  CAN_L  -----+    |
                                   |    |
                            [120Ω] |    |
                                   |    |
ArduPilot                          |    |
CAN_H         ---------------------+----+
CAN_L         ---------------------+
```

**重要**: 终端电阻必须接在总线两端！

---

## Step 2: 编译和烧录（1 分钟）

```bash
# 打开项目目录
cd f:/opensource/usv_esp32/esp32s3rover/ardupilot_rover_esp32s3_idf

# 编译
idf.py build

# 烧录（替换 COMX 为你的端口）
idf.py -p COMX flash monitor
```

**预期输出**:
```
...
CAN0: Initialized at 1000000 bps (mode=0)
CAN0: TWAI driver installed (TX=GPIO47, RX=GPIO38)
CAN0: TWAI alerts configured
CAN0: Timings - quanta/bit: 10, sample point: 87.5%
...
```

如果看到这些信息，恭喜！CAN 初始化成功！

---

## Step 3: 配置飞控（1 分钟）

### 使用 Mission Planner

1. 连接飞控到 Mission Planner
2. 进入 `CONFIG/TUNING` -> `Full Parameter List`
3. 设置以下参数:

```
CAN_D1_PROTOCOL = 1        # DroneCAN
CAN_D1_BITRATE = 1000000   # 1 Mbps
```

4. 点击 `Write Params`
5. 重启飞控

---

## Step 4: 验证通信（1 分钟）

### 方法 1: Mission Planner

1. 进入 `SETUP` -> `Optional Hardware` -> `DroneCAN/UAVCAN`
2. 点击 `Search for nodes`
3. 应该看到 Node 125 出现（或你配置的 Node ID）

```
Node ID: 125
Name: esp32s3-rover-dronecan
Status: OK
Uptime: 10s
```

### 方法 2: 串口监控

```bash
idf.py monitor

# 查找类似输出:
DroneCAN: Node 125 online
CAN0: TX success=10, RX received=5
```

### 方法 3: LED 指示（如果有）

如果你的 ESP32-S3 板有 LED:
- **常亮/闪烁**: 正常通信
- **熄灭**: 未初始化或总线故障

---

## 快速故障排查

### 问题 1: "CAN0: TWAI init failed"

**检查**:
- GPIO 引脚是否正确（TX=47, RX=38）
- SN65HVD230 供电是否正常（测量 VCC = 3.3V）
- 重新烧录固件

### 问题 2: "No messages received"

**检查**:
- 终端电阻是否安装（两端各 120Ω）
- CAN_H 和 CAN_L 是否接反
- 飞控 CAN 参数是否配置正确
- 用万用表测量 CAN_H 和 CAN_L 之间电压（应为 ~2.5V）

### 问题 3: "Bus-off detected"

**检查**:
- CAN 线缆质量（尽量用短线，< 1m）
- 终端电阻值（必须是 120Ω）
- 波特率是否匹配（ESP32 和飞控都是 1Mbps）

---

## 下一步

### 进阶测试
参见 `DRONECAN_TESTING_GUIDE.md` 进行全面测试

### 性能调优
参见 `DRONECAN_IMPLEMENTATION_PLAN.md` 了解优化策略

### 故障诊断
参见 `DRONECAN_TESTING_GUIDE.md` 第 6 节

---

## 常见应用场景

### 场景 1: 远程 ID 广播（参考 ArduRemoteID）

ESP32-S3 从飞控获取位置信息，通过蓝牙/WiFi 广播 Remote ID。

**优势**: 专用硬件，不占用飞控资源。

### 场景 2: 传感器节点（USV 应用）

ESP32-S3 接入深度计、水质传感器等，通过 DroneCAN 发送到飞控。

**配置**:
- UART2: DST800 深度计（4800 baud）
- CAN: 发送深度数据到飞控（10 Hz）

### 场景 3: 执行器控制

ESP32-S3 接收飞控 CAN 命令，控制舵机、电调等。

**优势**: 长距离通信（CAN 可达 40m @ 1Mbps）。

---

## 性能预期

基于 **ArduRemoteID** 实测数据:

| 指标 | 预期值 |
|------|--------|
| 初始化时间 | < 500ms |
| 延迟 | < 5ms |
| 吞吐量 | 500 msg/s (RX), 200 msg/s (TX) |
| CPU 占用 | < 10% @ 1Mbps |
| 内存占用 | < 1MB |

---

## 获取帮助

### 文档
- `DRONECAN_IMPLEMENTATION_SUMMARY.md`: 完整总结
- `DRONECAN_IMPLEMENTATION_PLAN.md`: 技术细节
- `DRONECAN_TESTING_GUIDE.md`: 详细测试

### 参考代码
- ArduRemoteID: `f:\opensource\usv_esp32\ArduRemoteID-master\`
- 本项目: `libraries/AP_HAL_ESP32/CANIface.cpp`

### 在线资源
- ESP-IDF TWAI 文档: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html
- DroneCAN 规范: https://dronecan.github.io/
- ArduPilot 论坛: https://discuss.ardupilot.org/

---

**祝您使用愉快！如有问题，请参考详细文档或提交 Issue。**

**最后更新**: 2025-10-27
