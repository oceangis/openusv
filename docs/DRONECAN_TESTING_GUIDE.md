# ESP32-S3 DroneCAN 测试验证指南

基于 ArduRemoteID 项目的实战经验，提供完整的 DroneCAN 测试流程。

## 1. 编译和烧录

### 1.1 编译配置

```bash
cd f:/opensource/usv_esp32/esp32s3rover/ardupilot_rover_esp32s3_idf

# 清理旧构建
idf.py fullclean

# 配置硬件定义（选择 esp32s3rover）
idf.py menuconfig
# Component config -> ArduPilot -> Board Type -> esp32s3rover

# 编译
idf.py build

# 烧录
idf.py -p COM_PORT flash monitor
```

### 1.2 验证编译结果

检查编译输出中是否包含：
```
...
Building ESP-IDF components...
[100%] Built target AP_HAL_ESP32
...
CAN driver: ENABLED
HAL_NUM_CAN_IFACES: 1
DroneCAN: ENABLED
...
```

## 2. 硬件连接测试

### 2.1 CAN 总线连接

**硬件要求**:
- ESP32-S3 开发板（16MB Flash, 8MB PSRAM）
- SN65HVD230 CAN 收发器（或兼容型号）
- CAN 总线终端电阻（120Ω，两端各一个）
- ArduPilot 飞控（或其他 DroneCAN 设备）

**接线**:
```
ESP32-S3              SN65HVD230        CAN Bus
GPIO 47 (TX) -------> TXD
GPIO 38 (RX) <------- RXD
GND          -------> GND
3.3V         -------> VCC
                      CANH ----------> CAN_H (to bus)
                      CANL ----------> CAN_L (to bus)
```

**终端电阻**:
```
CAN_H ---[120Ω]--- CAN_L  (两端都需要)
```

### 2.2 串口监控

```bash
idf.py monitor

# 期望输出:
CAN0: Initialized at 1000000 bps (mode=0)
CAN0: TWAI driver installed (TX=GPIO47, RX=GPIO38)
CAN0: TWAI alerts configured
CAN0: Timings - quanta/bit: 10, sample point: 87.5%
```

## 3. 功能测试

### 3.1 测试 1: CAN 初始化

**目标**: 验证 TWAI 驱动正确初始化

**操作**:
1. 上电 ESP32-S3
2. 连接串口监控

**预期结果**:
```
CAN0: Initialized at 1000000 bps
CAN0: TWAI driver installed
```

**失败排查**:
- 检查 GPIO 引脚是否正确配置
- 检查 CAN 收发器供电（3.3V）
- 测量 CAN_H 和 CAN_L 之间电压（应为 2.5V）

### 3.2 测试 2: CAN 发送

**目标**: 验证能够发送 DroneCAN 消息

**工具**: Mission Planner + SLCAN 适配器（或逻辑分析仪）

**操作**:
1. 连接 ArduPilot 飞控到 CAN 总线
2. 在 Mission Planner 中启用 DroneCAN
3. 观察 ESP32-S3 是否发送心跳

**预期结果** (Mission Planner):
```
DroneCAN: Node 125 detected
NodeStatus: uptime=10s, health=OK
```

**预期结果** (ESP32-S3 串口):
```
CAN0: TX success (ID=0x10F0007D, DLC=7)
stats: tx_success=10
```

**失败排查**:
- 检查 `HAL_NUM_CAN_IFACES` 是否为 1
- 检查 DroneCAN 参数（CAN_D1_PROTOCOL = 1）
- 验证终端电阻是否正确

### 3.3 测试 3: CAN 接收

**目标**: 验证能够接收 DroneCAN 消息

**操作**:
1. 从飞控发送 GetNodeInfo 请求
2. 观察 ESP32-S3 是否响应

**预期结果** (Mission Planner):
```
Node 125 Info:
  Name: esp32s3-rover-dronecan
  Hardware: v1.0
  Software: v4.x.x
```

**预期结果** (ESP32-S3 串口):
```
DroneCAN: got GetNodeInfo request
CAN0: RX received (ID=0x1E00017D, DLC=0)
stats: rx_received=1
```

**失败排查**:
- 检查接收队列大小（rx_queue_len = 50）
- 检查过滤器配置
- 验证 CAN 收发器方向控制

### 3.4 测试 4: 高负载测试

**目标**: 验证在繁忙 CAN 总线上不丢帧

**操作**:
1. 配置飞控发送高速 ESC 命令（50 Hz）
2. 同时发送 RemoteID 消息（10 Hz）
3. 监控 ESP32-S3 统计

**预期结果** (1 分钟后):
```
CAN0: TX req:600 suc:600 rej:0 ovf:0
      RX rcv:3000 ovf:0 err:0
      Bus-off:0
```

**失败排查**:
- 如果 `rx_ovf > 0`: 增加 HAL_CAN_RX_QUEUE_SIZE
- 如果 `tx_ovf > 0`: 增加 HAL_CAN_TX_QUEUE_SIZE
- 如果 `Bus-off > 0`: 检查终端电阻和线缆质量

### 3.5 测试 5: 过滤器测试

**目标**: 验证优先级过滤器工作正常

**操作**:
1. 记录默认过滤器配置下的 CPU 使用率
2. 修改过滤器为 accept-all
3. 对比 CPU 使用率

**ArduRemoteID 经验**:
- 优先级过滤: CPU < 10% @ 1Mbps 繁忙总线
- Accept-all: CPU > 80% @ 1Mbps 繁忙总线

**配置修改** (hwdef.dat):
```c
// 默认：优先级过滤
acceptance_code = 0x10000000U << 3;
acceptance_mask = 0x0FFFFFFFU << 3;

// Accept-all（仅用于测试）
acceptance_code = 0;
acceptance_mask = 0xFFFFFFFF;
```

### 3.6 测试 6: Bus-Off 恢复

**目标**: 验证总线故障自动恢复

**操作**:
1. 断开 CAN 总线（拔掉一根线）
2. 等待 5 秒
3. 重新连接总线

**预期结果**:
```
CAN0: Bus state: TWAI_STATE_BUS_OFF
CAN0: Initiating bus-off recovery
[2 seconds later]
CAN0: Bus recovered, state: TWAI_STATE_RUNNING
```

**失败排查**:
- 检查 `recoverFromBusOff()` 逻辑
- 验证 2 秒恢复间隔
- 确认 TWAI_ALERT_BUS_OFF 已启用

## 4. 性能基准测试

### 4.1 延迟测试

**工具**: 逻辑分析仪 + CAN 解码

**测试方法**:
1. 从飞控发送 GetNodeInfo
2. 测量接收到响应发送的延迟

**ArduRemoteID 基准**:
- 平均延迟: < 2ms
- 最大延迟: < 5ms

### 4.2 吞吐量测试

**测试方法**:
1. 飞控以最大速率发送消息
2. ESP32-S3 统计接收数量

**ArduRemoteID 基准**:
- RX 吞吐: 500 msg/s (无丢失)
- TX 吞吐: 200 msg/s

### 4.3 内存使用

**测试方法**:
```cpp
// 在代码中添加:
hal.console->printf("Heap free: %lu bytes\n", esp_get_free_heap_size());
hal.console->printf("PSRAM free: %lu bytes\n", esp_get_free_psram_size());
```

**ArduRemoteID 基准**:
- Heap 使用: < 50KB
- PSRAM 使用: < 128KB（CAN 缓冲区）

## 5. 与 ArduRemoteID 对比测试

### 5.1 对比测试环境

| 项目 | ArduRemoteID | ArduPilot ESP32-S3 |
|------|--------------|---------------------|
| 平台 | Arduino-ESP32 | ESP-IDF 5.5.1 |
| TWAI 配置 | 相同 | 相同 |
| 过滤器 | 优先级 | 优先级 |
| 队列深度 | RX=50, TX=5 | RX=128, TX=32 |

### 5.2 功能对比

| 功能 | ArduRemoteID | 预期结果 |
|------|--------------|---------|
| 初始化成功率 | 100% | 100% |
| 发送成功率 | 99.9% | > 99.9% |
| 接收成功率 | 99.5% | > 99.5% |
| Bus-Off 恢复 | < 2s | < 2s |
| CPU 占用 | < 10% | < 10% |

## 6. 常见问题排查

### 6.1 编译错误

**错误**: `CANIface.h: No such file or directory`

**解决**:
```bash
# 确认文件存在
ls libraries/AP_HAL_ESP32/CANIface.h
ls libraries/AP_HAL_ESP32/CANIface.cpp

# 检查 CMakeLists.txt
grep "CANIface" libraries/AP_HAL_ESP32/CMakeLists.txt
```

**错误**: `HAL_NUM_CAN_IFACES undeclared`

**解决**: 检查 hwdef.dat
```
define HAL_NUM_CAN_IFACES 1
```

### 6.2 运行时错误

**错误**: `CAN0: TWAI init failed`

**排查**:
1. 检查 GPIO 配置
2. 验证 SN65HVD230 供电
3. 测试 TWAI 驱动单独初始化

**错误**: `CAN0: No messages received`

**排查**:
1. 检查 CAN 总线终端电阻
2. 验证飞控 CAN 配置（bitrate, protocol）
3. 使用逻辑分析仪检查 CAN_H/CAN_L 波形

**错误**: `Bus-off detected, cannot recover`

**排查**:
1. 检查 CAN 线缆质量（长度 < 1m 用于测试）
2. 验证终端电阻值（120Ω）
3. 降低波特率测试（500 kbps）

### 6.3 性能问题

**症状**: CPU 占用过高（> 50%）

**排查**:
1. 检查是否启用优先级过滤
2. 验证 `rx_queue_len = 50`（硬件队列）
3. 检查是否有过多调试打印

**症状**: 消息丢失（rx_ovf > 0）

**解决**:
```cpp
// 增加队列深度
#define HAL_CAN_RX_QUEUE_SIZE 256  // 从 128 增加到 256
```

## 7. 生产验证清单

### 7.1 功能验证

- [ ] CAN 初始化成功
- [ ] 发送心跳（1 Hz）
- [ ] 响应 GetNodeInfo
- [ ] 动态节点分配（DNA）
- [ ] 参数读写（GetSet）
- [ ] Bus-Off 自动恢复

### 7.2 性能验证

- [ ] 延迟 < 5ms
- [ ] RX 吞吐 > 500 msg/s
- [ ] TX 吞吐 > 200 msg/s
- [ ] CPU 占用 < 10%
- [ ] 内存使用 < 1MB

### 7.3 可靠性验证

- [ ] 24 小时稳定运行
- [ ] 1000 次 Bus-Off 恢复测试
- [ ] 温度循环测试（-20°C ~ 60°C）
- [ ] 电源波动测试（3.0V ~ 3.6V）

### 7.4 兼容性验证

- [ ] 与 ArduPilot 飞控通信
- [ ] 与 UAVCAN GUI Tool 通信
- [ ] 与 Mission Planner 通信
- [ ] 多节点总线（> 10 个节点）

## 8. 调试工具

### 8.1 推荐工具

1. **Mission Planner**: DroneCAN 可视化
2. **UAVCAN GUI Tool**: 底层 DroneCAN 调试
3. **SLCAN 适配器**: CAN 总线监控
4. **逻辑分析仪**: 硬件波形分析（推荐 Saleae）
5. **ESP-IDF Monitor**: 实时日志查看

### 8.2 调试命令

```bash
# ESP-IDF 监控（带过滤）
idf.py monitor | grep CAN

# 查看实时统计
# 在代码中添加定时打印:
void printStats() {
    hal.console->printf("CAN Stats: TX=%lu RX=%lu ERR=%lu\n",
                       stats_.tx_success,
                       stats_.rx_received,
                       stats_.rx_errors);
}
```

### 8.3 ArduRemoteID 对比脚本

```python
# compare_can_stats.py
import serial
import time

# ArduRemoteID 串口
rid_port = serial.Serial('COM3', 115200)

# ArduPilot ESP32-S3 串口
ap_port = serial.Serial('COM4', 115200)

while True:
    rid_line = rid_port.readline().decode()
    ap_line = ap_port.readline().decode()

    if 'CAN' in rid_line:
        print(f"RID: {rid_line.strip()}")
    if 'CAN' in ap_line:
        print(f"AP:  {ap_line.strip()}")

    time.sleep(0.1)
```

## 9. 成功标准

### 9.1 基本成功标准

达到 ArduRemoteID 同等性能:
- ✅ 初始化成功率 100%
- ✅ 消息发送成功率 > 99%
- ✅ CPU 占用 < 10%
- ✅ Bus-Off 恢复 < 2s

### 9.2 扩展成功标准

超越 ArduRemoteID:
- 🎯 更大队列（RX=128 vs 50）
- 🎯 更好错误统计
- 🎯 支持 configureFilters() API
- 🎯 集成 ArduPilot 日志系统

## 10. 下一步计划

### 10.1 短期（1 周）

1. 完成基本功能测试
2. 验证与飞控通信
3. 修复发现的 bug

### 10.2 中期（1 月）

1. 长期稳定性测试
2. 性能优化
3. 文档完善

### 10.3 长期（3 月）

1. 提交 PR 到 ArduPilot
2. 社区测试和反馈
3. 生产部署

---

**参考资料**:
- ArduRemoteID 项目: `f:\opensource\usv_esp32\ArduRemoteID-master\`
- ESP-IDF TWAI 文档: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html
- DroneCAN 规范: https://dronecan.github.io/
- ArduPilot CAN 文档: https://ardupilot.org/copter/docs/common-canbus-setup-advanced.html
