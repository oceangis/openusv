# LoRa MAVLink 双向遥测系统

## 概述

基于 SX1268 的 LoRa MAVLink 遥测链路，实现 ESP32-S3 无人船控制板与 EBYTE E22-400MBL-SC 地面站评估板之间的双向通信。LoRa 作为 ArduPilot 的虚拟 UART（SERIAL4）运行，提供低速率、长距离的 MAVLink 遥测和指令通道。

## 硬件连接

### ESP32-S3 控制板 — SX1268ZTR4-GC 模块

| 信号   | GPIO | 说明                        |
|--------|------|-----------------------------|
| SCK    | IO40 | SPI2 时钟                   |
| MISO   | IO39 | SPI2 主入从出               |
| MOSI   | IO41 | SPI2 主出从入               |
| CS     | IO42 | SPI 片选（软件控制）        |
| BUSY   | IO2  | SX1268 忙信号               |
| DIO1   | IO1  | 中断输出                    |
| RST    | NC   | 未连接（使用软件复位/唤醒） |
| DIO2   | NC   | 未连接（设为 RF Switch 无影响）|
| DIO3   | NC   | 未连接（使用晶振，非 TCXO） |

**关键硬件特征：**
- 模块使用晶振（非 TCXO），无需配置 DIO3
- RST 引脚未接 GPIO，上电后通过 SPI 发送 GetStatus(0xC0) 唤醒
- SPI2_HOST，8MHz 时钟，Mode 0

### EBYTE E22-400MBL-SC 评估板

- 内部使用 TCXO（DIO3 = 3.3V 控制）
- DIO2 RF Switch = false（评估板内部不使用 DIO2 做射频切换）
- USB CDC 接口，用于 LoRa-USB 桥接

## RF 参数配置

两端参数必须完全匹配，否则无法解调：

| 参数             | 值              | 说明                          |
|------------------|-----------------|-------------------------------|
| 频率             | 433 MHz         | E22-400M 系列频段             |
| 扩频因子 (SF)    | 7               | 低延迟，2km+ 水面覆盖         |
| 带宽 (BW)        | 500 kHz         | 最大带宽，~11kbps 原始速率    |
| 编码率 (CR)      | 4/5             | 最低冗余，最大吞吐            |
| LDRO             | 关闭            | SF7/BW500 符号时间 0.256ms，无需 LDRO |
| 前导码长度       | 8 符号          |                               |
| 报头模式         | 显式 (Explicit) | 接收端从报头读取载荷长度      |
| CRC              | 开启            |                               |
| IQ 极性          | 正常（不反转）  |                               |
| Sync Word        | 0x1444          | EBYTE 风格（0x14 扩展为 0x1444）|
| 发射功率         | 22 dBm          | SX1268 最大值                 |
| PA 过流保护      | 0x38 (140mA)    | 22dBm 推荐值                  |

### Sync Word 说明

EBYTE 库调用 `sx126x_set_lora_sync_word(0x14)` 内部实现为读取寄存器 0x0740-0x0741，保留低 nibble，设置高 nibble，结果为 0x1444。ESP32 端直接写入 0x1444 到相同寄存器。

### 空中时间参考（SF7/BW500）

| 载荷大小 | 空中时间  |
|----------|-----------|
| 12 字节  | ~5 ms     |
| 32 字节  | ~7 ms     |
| 50 字节  | ~10 ms    |
| 100 字节 | ~16 ms    |
| 255 字节 | ~37 ms    |

SF7/BW500 相比 SF11/BW500 空中时间缩短约 30 倍，延迟从数百毫秒降至数十毫秒，适合遥控指令的实时响应。理论覆盖 2-5km（水面环境更优）。

## 软件架构

### 文件结构

```
components/lora_mavlink/
├── include/
│   └── lora_mavlink.h      # 公共接口、引脚定义、RF 参数
├── src/
│   ├── lora_mavlink.c       # 状态机、环形缓冲区、MAVLink 解析
│   ├── sx126x.c             # SX1268 寄存器级驱动
│   ├── sx126x_hal.h         # HAL 抽象层头文件（含命令/寄存器定义）
│   └── sx126x_hal_esp32.c   # ESP32 SPI HAL 实现
└── CMakeLists.txt
```

### 初始化流程

```
sx126x_hal_init()           → SPI2 总线 + GPIO 配置
SPI wakeup (GetStatus)      → 绕过 BUSY 检查唤醒芯片
sx126x_set_standby(RC)      → 进入待机模式
sx126x_set_packet_type(LoRa)
sx126x_set_regulator_mode(DCDC)
sx126x_calibrate(0x7F)      → 全模块校准
sx126x_calibrate_image()    → 433MHz 镜像抑制校准
sx126x_set_dio2_rf_switch(true)
sx126x_set_buffer_base_address(TX=0, RX=128)
sx126x_set_pa_config() + OCP + TX params
sx126x_set_rf_frequency(433MHz)
sx126x_set_modulation_params(SF7, BW500, CR4/5, LDRO=0)
sx126x_set_packet_params(preamble=8, explicit, 255, CRC=on, IQ=normal)
sx126x_set_sync_word(0x1444)
sx126x_set_dio_irq_params()  → DIO1: TX_DONE | RX_DONE | CRC_ERR | TIMEOUT | CAD
```

### RX 主导状态机

LoRa 是半双工的——同一时刻只能发送或接收。作为无人船控制链路，LoRa 同时承担遥测下行和遥控上行（替代传统 FrSky 遥控器），因此必须保证大部分时间处于 RX 状态以接收控制指令。

**调度策略：RX 主导，定时遥测 TX**

```
 ←————————— 持续 RX（默认状态）——————————→←TX→←————— 持续 RX ——————→
 ┌──────────────────────────────────────┐┌──┐┌──────────────────────┐
 │ 接收控制指令（油门/转向/MANUAL_CONTROL）││10ms││ 继续接收              │
 │ 随时响应上行数据                      ││遥测││                      │
 └──────────────────────────────────────┘└──┘└──────────────────────┘
                                          ↑
                                     每 500ms 一次
```

- 默认状态为 RX（持续监听上行指令）
- `LORA_TELEM_INTERVAL_MS = 500`：每 500ms 发送一个遥测包（2Hz）
- `LORA_TX_MAX_BYTES = 50`：单包上限 50 字节（SF7/BW500 空中时间 ~10ms）
- TX 仅在有待发数据且到达发送间隔时才执行
- TX 完成后立即切回 RX
- 有效 RX 占空比 > 98%（10ms TX / 500ms 周期）

### MAVLink 优先级路由

TX 数据经过 MAVLink v2 解析器，按消息 ID 分流到两个缓冲区：

| 缓冲区   | 大小    | 消息类型                                              |
|----------|---------|-------------------------------------------------------|
| tx_hi    | 512B    | HEARTBEAT, PARAM_VALUE, COMMAND_ACK, STATUSTEXT 等    |
| tx_lo    | 1024B   | 其他遥测消息（GPS, ATTITUDE, ...）                    |

发送时高优先级缓冲区优先。低优先级缓冲区满时丢弃最旧数据，保持流动。

### IRQ 处理

SX1268 的 DIO1 脉冲宽度极短（<1ms），1ms 轮询周期可能漏掉。因此每次循环直接读取 IRQ 状态寄存器（`GetIrqStatus` 命令），不依赖 DIO1 电平。

处理的 IRQ 事件：
- `TX_DONE` → 更新统计，状态回 IDLE
- `RX_DONE` → 读取数据到 RX 环形缓冲区
- `CRC_ERR` → 丢弃，计数器 +1
- `RX_TX_TIMEOUT` → 状态回 IDLE
- `CAD_DONE` → 信道检测结果

### SX1268 无 RST 引脚唤醒

由于 RST 未接 GPIO，上电或异常后无法硬件复位。使用 SPI 软件唤醒：

```c
// 绕过 BUSY 检查发送 GetStatus
uint8_t tx[2] = {0xC0, 0x00};
sx126x_hal_spi_transfer_no_busy(tx, rx, 2);
// 然后等待 BUSY 拉低（最多 1500ms）
sx126x_hal_wait_busy(1500);
```

**注意：如果芯片被误写入深度睡眠（`SetSleep(0x00)`），无 RST 引脚时只能断电重启恢复。**

## 调试经验

### 问题 1：上行始终 rx=0（已解决）

**现象**：ESP32→E22 下行正常，E22→ESP32 上行 rx 始终为 0。

**根因**：ArduPilot 持续产生 MAVLink 数据（特别是 HEARTBEAT 每秒一次），TX-dominant 调度导致芯片几乎全部时间在 TX 模式，无法接收上行数据。

**修复**：
1. 状态机从 TX 主导改为 RX 主导（默认 RX，定时 TX）
2. `sx126x_start_rx_continuous()` 中重置 `SetPacketParams` 的 payload_len 为 255（TX 会将其改为实际发送长度，影响 RX 最大接收长度）
3. LDRO 计算修正：基于符号时间（2^SF/BW_Hz）是否 > 16ms，而非硬编码 SF>=10

### 问题 2：SX1268 深度睡眠锁死（已解决）

**现象**：误调用 `sx126x_set_sleep(0x00)` 后，BUSY 永远为高，SPI 无响应。

**根因**：深度睡眠需要 RST 引脚或 SPI CS 下降沿唤醒。由于 SPI 驱动已接管 GPIO，bit-bang 唤醒无效；SPI 事务的 CS 信号也无法唤醒（时序不满足要求）。

**修复**：断电重启。代码中已移除所有 `sx126x_set_sleep` 调用（除 deinit 外）。

### 问题 3：TX_DONE IRQ 在 RX 状态下重复触发

**现象**：日志显示 `IRQ detected: 0x0001 (state=2)`，即 TX_DONE 在 RX 模式下触发。

**分析**：TX 完成后 IRQ 被清除并进入 RX，但下一个循环仍读到 TX_DONE。实际上是快速的 TX→IDLE→RX 状态转换在同一循环内完成（process_irq 改变状态后，状态机立即执行 start_rx），日志中的 state=2 是 process_irq 执行时的瞬时状态。不影响功能。

## 测试结果

### 双向通信验证 v1 — SF11（2026-03-07）

测试环境：ESP32-S3 控制板 ↔ E22-400MBL-SC 评估板，室内近距离，SF11/BW500

**上行（E22→ESP32）**：
- 发送 6 个测试包（间隔 3 秒），接收 4 个（67%）
- ArduPilot 稳定后，后续 4/4 = 100% 接收率
- 信号质量：RSSI = -3 ~ -5 dBm, SNR = 7 ~ 8 dB

### 双向通信验证 v2 — SF7 + RX 主导（2026-03-07）

测试环境：ESP32-S3 控制板 ↔ E22-400MBL-SC 评估板，室内近距离，SF7/BW500

**上行（E22→ESP32）**：
- 发送 8 个测试包，接收 8 个（**100%**）
- 信号质量：RSSI = -6 ~ -11 dBm, SNR = 12 ~ 13 dB

**下行（ESP32→E22）**：
- 持续发送 MAVLink 遥测数据，2Hz（每 500ms 一包）
- E22 接收 57 个包（5 秒内约 10 包，符合设计）
- E22 通过 USB CDC 输出 0xFD 开头的 MAVLink v2 帧

**结论**：SF7 + RX 主导状态机实现 100% 上行接收率，TX 空中时间 ~10ms/包，RX 占空比 >98%。

### 统计日志格式

```
Stats: tx=21 rx=4 crc=0 st=2 cm=6 irq=0x0000 dio1=0 txhi=0 txlo=924 rxbuf=0
```

| 字段   | 含义                                  |
|--------|---------------------------------------|
| tx     | 已发送包数                            |
| rx     | 已接收包数                            |
| crc    | CRC 错误计数                          |
| st     | 软件状态 (0=IDLE,1=RX,2=TX,3=CAD)   |
| cm     | 芯片模式 (5=RX,6=TX)                |
| irq    | 当前 IRQ 寄存器值                     |
| dio1   | DIO1 引脚电平                         |
| txhi   | 高优先级 TX 缓冲区字节数             |
| txlo   | 低优先级 TX 缓冲区字节数             |
| rxbuf  | RX 缓冲区字节数                       |

## 配置参数速查

所有参数定义在 `components/lora_mavlink/include/lora_mavlink.h`：

```c
// 引脚
LORA_PIN_SCK   = 40
LORA_PIN_MISO  = 39
LORA_PIN_MOSI  = 41
LORA_PIN_CS    = 42
LORA_PIN_RST   = -1   // 未连接
LORA_PIN_BUSY  = 2
LORA_PIN_DIO1  = 1

// RF 参数
LORA_FREQ_HZ   = 433000000
LORA_TX_POWER  = 22        // dBm
LORA_SF        = 7         // 低延迟，2km+ 水面
LORA_BW        = 500       // kHz

// RX 主导调度
LORA_TELEM_INTERVAL_MS = 500  // 遥测发送间隔（2Hz）
LORA_TX_MAX_BYTES      = 50   // 单包上限（~10ms 空中时间）

// 缓冲区
LORA_RX_BUFFER_SIZE    = 512
LORA_TX_BUFFER_SIZE    = 1024
LORA_TX_HI_BUFFER_SIZE = 512
```
