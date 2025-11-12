# ArduPilot ESP32-S3 DroneCAN 完整实现方案

基于 ArduRemoteID 项目的成熟实现，为 ArduPilot ESP32-S3 Rover 提供生产就绪的 DroneCAN 支持。

## 1. ArduRemoteID 项目深度分析

### 1.1 项目概述
- **路径**: `f:\opensource\usv_esp32\ArduRemoteID-master\`
- **平台**: Arduino-ESP32 (ESP-IDF 底层)
- **目标**: OpenDroneID Remote ID 模块
- **成功案例**: 多款商业产品（BlueMark DB210, CUAV C-RID 等）

### 1.2 核心架构分析

#### 1.2.1 CANDriver 类 (底层 TWAI 驱动封装)

**文件**: `RemoteIDModule/CANDriver.h` & `CANDriver.cpp`

**关键设计**:
```cpp
class CANDriver {
    // ESP-IDF TWAI 驱动封装
    void init(uint32_t bitrate, uint32_t acceptance_code, uint32_t acceptance_mask);
    bool send(const CANFrame &frame);
    bool receive(CANFrame &out_frame);

private:
    bool computeTimings(uint32_t target_bitrate, Timings& out_timings);
    void init_once(bool enable_irq);
};
```

**TWAI 配置** (line 36-52):
```cpp
// 1 Mbps CAN 时序配置
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

// 接受所有帧（后续可配置过滤器）
static twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// 通用配置
static const twai_general_config_t g_config = {
    .mode = TWAI_MODE_NORMAL,
    .tx_io = PIN_CAN_TX,           // GPIO 47 (ESP32-S3)
    .rx_io = PIN_CAN_RX,           // GPIO 38 (ESP32-S3)
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = 5,             // 发送队列深度
    .rx_queue_len = 50,            // 接收队列深度 (重要!)
    .alerts_enabled = TWAI_ALERT_NONE,
    .clkout_divider = 0,
    .intr_flags = ESP_INTR_FLAG_LEVEL2  // 中断优先级
};
```

**初始化流程** (line 54-84):
```cpp
void CANDriver::init_once(bool enable_irq) {
    // 1. 安装 TWAI 驱动
    twai_driver_install(&g_config, &t_config, &f_config);

    // 2. 重新配置告警（仅接收相关）
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL;
    twai_reconfigure_alerts(alerts_to_enable, NULL);

    // 3. 启动 TWAI 驱动
    twai_start();
}
```

**发送逻辑** (line 221-258):
```cpp
bool CANDriver::send(const CANFrame &frame) {
    // 1. 转换 CANFrame -> twai_message_t
    twai_message_t message {};
    message.identifier = frame.id;
    message.extd = frame.isExtended() ? 1 : 0;
    message.data_length_code = frame.dlc;
    memcpy(message.data, frame.data, 8);

    // 2. 检查总线状态并自动恢复
    twai_status_info_t info {};
    twai_get_status_info(&info);
    switch (info.state) {
        case TWAI_STATE_STOPPED:
            twai_start();
            break;
        case TWAI_STATE_BUS_OFF:
            // 每 2 秒尝试恢复
            if (now - last_bus_recovery_ms > 2000) {
                last_bus_recovery_ms = now;
                twai_initiate_recovery();
            }
            break;
    }

    // 3. 发送（5ms 超时）
    const esp_err_t sts = twai_transmit(&message, pdMS_TO_TICKS(5));
    return (sts == ESP_OK);
}
```

**接收逻辑** (line 260-278):
```cpp
bool CANDriver::receive(CANFrame &out_frame) {
    twai_message_t message {};

    // 5ms 超时接收
    esp_err_t recverr = twai_receive(&message, pdMS_TO_TICKS(5));
    if (recverr != ESP_OK) {
        return false;
    }

    // 转换 twai_message_t -> CANFrame
    memcpy(out_frame.data, message.data, 8);
    out_frame.dlc = message.data_length_code;
    out_frame.id = message.identifier;
    if (message.extd) {
        out_frame.id |= CANARD_CAN_FRAME_EFF;  // 扩展帧标志
    }

    return true;
}
```

#### 1.2.2 DroneCAN 类 (协议层)

**文件**: `RemoteIDModule/DroneCAN.h` & `DroneCAN.cpp`

**关键设计**:
```cpp
class DroneCAN : public Transport {
    CANDriver can_driver;                  // TWAI 驱动封装
    CanardInstance canard;                 // libcanard 实例
    uint32_t canard_memory_pool[CAN_POOL_SIZE/sizeof(uint32_t)];  // 4KB 内存池

public:
    void init(void) override;
    void update(void) override;

private:
    void processTx(void);                  // 处理发送队列
    void processRx(void);                  // 处理接收队列
    void node_status_send(void);           // 发送心跳
    bool do_DNA(void);                     // 动态节点分配
};
```

**重要优化 - 接受过滤器** (line 55-73):
```cpp
void DroneCAN::init(void) {
    /*
     * ESP32 TWAI 性能限制优化策略:
     *
     * 问题: ESP32 TWAI 栈效率低，忙碌总线会耗尽 CPU
     *
     * 解决: 使用接受过滤器阻止高速率消息（如 ESC 命令）
     *       ESP32 过滤器简单，只能使用优先级位过滤
     *
     * 策略: 设置优先级位接受码 = 0x10000000 << 3
     *       只接收优先级 >= 16 的消息 (CANARD_TRANSFER_PRIORITY_MEDIUM 或更低)
     *       高速率消息（优先级 < 16）被硬件自动丢弃
     */
    const uint32_t acceptance_code = 0x10000000U << 3;
    const uint32_t acceptance_mask = 0x0FFFFFFFU << 3;

    can_driver.init(1000000, acceptance_code, acceptance_mask);

    // 初始化 Canard
    canardInit(&canard, (uint8_t *)canard_memory_pool, sizeof(canard_memory_pool),
               onTransferReceived_trampoline, shouldAcceptTransfer_trampoline, NULL);

    // 设置节点 ID（从参数读取或使用 DNA）
    if (g.can_node > 0 && g.can_node < 128) {
        canardSetLocalNodeID(&canard, g.can_node);
    }
}
```

**更新循环** (line 83-95):
```cpp
void DroneCAN::update(void) {
    // 1. DNA（动态节点分配）- 如果没有 Node ID
    if (do_DNA()) {
        // 2. 每秒发送心跳和臂状态
        const uint32_t now_ms = millis();
        if (now_ms - last_node_status_ms >= 1000) {
            last_node_status_ms = now_ms;
            node_status_send();
            arm_status_send();
        }
    }

    // 3. 处理发送和接收
    processTx();
    processRx();
}
```

**发送处理** (line 244-265):
```cpp
void DroneCAN::processTx(void) {
    // 从 Canard 发送队列取帧
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        CANFrame txmsg {};
        txmsg.dlc = CANFrame::dataLengthToDlc(txf->data_len);
        memcpy(txmsg.data, txf->data, txf->data_len);
        txmsg.id = (txf->id | CANFrame::FlagEFF);  // 强制扩展帧

        // 发送成功则弹出队列
        if (can_driver.send(txmsg)) {
            canardPopTxQueue(&canard);
            tx_fail_count = 0;
        } else {
            // 失败则重试（最多 8 次）
            if (tx_fail_count < 8) {
                tx_fail_count++;
            } else {
                canardPopTxQueue(&canard);  // 放弃该帧
            }
            break;
        }
    }
}
```

**接收处理** (line 267-290):
```cpp
void DroneCAN::processRx(void) {
    CANFrame rxmsg;

    // 批处理接收（每次最多 60 帧）
    uint8_t count = 60;
    while (count-- && can_driver.receive(rxmsg)) {
        // 转换为 Canard 帧
        CanardCANFrame rx_frame {};
        uint64_t timestamp = micros64();
        rx_frame.data_len = CANFrame::dlcToDataLength(rxmsg.dlc);
        memcpy(rx_frame.data, rxmsg.data, rx_frame.data_len);
        rx_frame.id = rxmsg.id;

        // 交给 Canard 处理
        int err = canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
}
```

#### 1.2.3 板级配置

**文件**: `RemoteIDModule/board_config.h`

**ESP32-S3 配置示例**:
```cpp
#ifdef BOARD_ESP32S3_DEV
#define PIN_CAN_TX GPIO_NUM_47
#define PIN_CAN_RX GPIO_NUM_38
#define WS2812_LED_PIN GPIO_NUM_48
#endif

#ifdef BOARD_CUAV_RID
#define PIN_CAN_TX GPIO_NUM_47
#define PIN_CAN_RX GPIO_NUM_38
#define PIN_CAN_nSILENT GPIO_NUM_1      // CAN 静默模式控制
#define PIN_CAN_TERM GPIO_NUM_37         // CAN 终端电阻控制
#define CAN_TERM_EN  LOW
#define CAN_APP_NODE_NAME "net.cuav.c-rid"
#endif
```

### 1.3 关键经验总结

#### 成功要素
1. **TWAI 队列深度**: rx_queue_len = 50 (关键！)
2. **硬件过滤**: 使用优先级过滤减少 CPU 负载
3. **总线恢复**: 自动检测 BUS_OFF 并恢复
4. **批处理接收**: 每次处理多帧提高效率
5. **超时控制**: 发送/接收都使用 5ms 超时

#### 已知限制
1. ESP32 TWAI 过滤器简单（只能按优先级）
2. 忙碌总线下 CPU 占用高
3. 不支持 CAN FD

---

## 2. ArduPilot HAL CANIface 适配设计

### 2.1 架构对比

| 组件 | ArduRemoteID | ArduPilot HAL | 适配策略 |
|------|--------------|---------------|---------|
| **CAN 驱动** | CANDriver (TWAI 封装) | ESP32::CANIface (HAL) | 复用 TWAI 配置，适配 HAL 接口 |
| **帧结构** | CANFrame (自定义) | AP_HAL::CANFrame (标准) | 完全兼容（结构相同） |
| **消息队列** | 直接调用 TWAI API | ObjectBuffer + ByteBuffer | 使用 ArduPilot 队列机制 |
| **线程模型** | Arduino loop() | FreeRTOS Task (Scheduler) | 在 Scheduler 线程中调用 |
| **中断处理** | TWAI 内部中断 | HAL ISR + 信号量 | 使用 BinarySemaphore 通知 |
| **过滤器** | 简单优先级过滤 | configureFilters() | 实现 HAL 接口，限制功能 |
| **错误处理** | 简单计数 | 详细统计 (bus_stats_t) | 完整实现统计 |

### 2.2 类层次结构

```
AP_HAL::CANIface (抽象基类)
    ├── ChibiOS::CANIface (STM32 实现)
    ├── Linux::CANSocketIface (Linux SocketCAN)
    └── ESP32::CANIface (我们的实现) ← 新增
            ├── TWAI 驱动封装 (from ArduRemoteID)
            ├── 队列管理 (HAL 标准)
            └── 中断处理 (FreeRTOS)
```

### 2.3 核心设计决策

#### 决策 1: 队列实现
- **ArduRemoteID**: 直接使用 TWAI 队列（硬件 + FreeRTOS Queue）
- **ArduPilot HAL**: ObjectBuffer + ByteBuffer（软件队列）
- **选择**: 混合方案
  - RX: TWAI 硬件队列 (50 深度) + HAL ObjectBuffer (128 深度)
  - TX: HAL 优先级队列 (按帧优先级排序)

#### 决策 2: 中断模型
- **ArduRemoteID**: TWAI 内部中断 + 轮询
- **ArduPilot HAL**: ISR + BinarySemaphore 唤醒
- **选择**: 使用 TWAI 告警 + 信号量通知 Scheduler

#### 决策 3: 过滤器策略
- **限制**: ESP32 TWAI 只支持简单过滤
- **实现**: configureFilters() 返回部分支持
  - 精确 ID 匹配: 最多 1 个
  - Mask 过滤: 使用优先级位（如 ArduRemoteID）

#### 决策 4: 内存分配
- **堆栈**: 在类内部（避免线程堆栈溢出）
- **队列**: 使用 8MB PSRAM（大容量缓冲）
- **Canard 池**: AP_CANManager 管理（非 CANIface 职责）

---

## 3. 完整实现代码

### 3.1 CANIface.h

**位置**: `libraries/AP_HAL_ESP32/CANIface.h`

```cpp
/*
 * ESP32-S3 CAN Interface Implementation
 * Based on ArduRemoteID CANDriver and ChibiOS::CANIface
 */

#pragma once

#include "AP_HAL_ESP32.h"

#if HAL_NUM_CAN_IFACES

#include <driver/twai.h>
#include <esp_err.h>

#ifndef HAL_CAN_RX_QUEUE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

#ifndef HAL_CAN_TX_QUEUE_SIZE
#define HAL_CAN_TX_QUEUE_SIZE 32
#endif

static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");

namespace ESP32 {

class CANIface : public AP_HAL::CANIface
{
public:
    CANIface(uint8_t index);

    // ========== HAL Interface Implementation ==========

    // Initialize CAN interface
    bool init(const uint32_t bitrate, const OperatingMode mode) override;

    // Send CAN frame
    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                 CanIOFlags flags) override;

    // Receive CAN frame
    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us,
                    CanIOFlags& out_flags) override;

    // Configure hardware filters
    bool configureFilters(const CanFilterConfig* filter_configs,
                          uint16_t num_configs) override;

    // Wait for read/write events
    bool select(bool &read_select, bool &write_select,
                const AP_HAL::CANFrame* const pending_tx,
                uint64_t timeout_us) override;

    // Set event semaphore for ISR notification
    bool set_event_handle(AP_HAL::BinarySemaphore *sem_handle) override;

    // Get number of available filters
    uint16_t getNumFilters() const override { return 1; }  // ESP32 limitation

    // Get error count
    uint32_t getErrorCount() const override;

    // Check if bus-off
    bool is_busoff() const override;

    // Check if initialized
    bool is_initialized() const override { return initialized_; }

    // Get statistics
    void get_stats(ExpandingString &str) override;
    const bus_stats_t *get_statistics(void) const override { return &stats_; }

    // Test methods
    void flush_tx() override;
    void clear_rx() override;

protected:
    int8_t get_iface_num() const override { return self_index_; }
    bool add_to_rx_queue(const CanRxItem &rx_item) override;

private:
    // ========== TWAI Driver Management ==========

    struct Timings {
        uint16_t prescaler;
        uint8_t sjw;
        uint8_t bs1;
        uint8_t bs2;
    };

    bool computeTimings(uint32_t target_bitrate, Timings& out_timings);
    bool initTWAI(uint32_t bitrate, uint32_t acceptance_code, uint32_t acceptance_mask);
    void setupTWAIAlerts();

    // ========== Rx/Tx Processing ==========

    void processTxQueue();
    void processRxQueue();
    bool recoverFromBusOff();
    void pollErrorFlags();

    // ========== Hardware Access ==========

    bool readFromTWAI(twai_message_t &message);
    bool writeToTWAI(const twai_message_t &message);
    void convertFrameToTWAI(const AP_HAL::CANFrame &frame, twai_message_t &message);
    void convertTWAIToFrame(const twai_message_t &message, AP_HAL::CANFrame &frame);

    // ========== Member Variables ==========

    const uint8_t self_index_;

    // State flags
    bool initialized_ : 1;
    bool busoff_detected_ : 1;

    // Rx buffer (HAL standard)
    CanRxItem rx_buffer_[HAL_CAN_RX_QUEUE_SIZE];
    ByteBuffer rx_bytebuffer_;
    ObjectBuffer<CanRxItem> rx_queue_;

    // Tx priority queue
    struct TxItem {
        CanTxItem item;

        bool operator<(const TxItem& rhs) const {
            return item < rhs.item;
        }
    };
    TxItem tx_queue_[HAL_CAN_TX_QUEUE_SIZE];
    uint8_t tx_queue_head_;
    uint8_t tx_queue_tail_;

    // Event notification
    AP_HAL::BinarySemaphore *event_sem_;

    // Timing
    uint64_t last_bus_recovery_us_;
    uint64_t last_error_poll_us_;

    // Statistics
    bus_stats_t stats_;

    // Filter configuration
    uint32_t acceptance_code_;
    uint32_t acceptance_mask_;
    bool filters_configured_;

    // TWAI hardware pins (from hwdef)
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
};

}  // namespace ESP32

#endif  // HAL_NUM_CAN_IFACES
```

### 3.2 CANIface.cpp (第1部分 - 初始化)

**位置**: `libraries/AP_HAL_ESP32/CANIface.cpp`

```cpp
/*
 * ESP32-S3 CAN Interface Implementation
 */

#include "CANIface.h"

#if HAL_NUM_CAN_IFACES

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "Scheduler.h"

extern const AP_HAL::HAL& hal;

// TWAI GPIO pins (should come from hwdef.dat)
#ifndef HAL_ESP32_CAN1_TX_PIN
#define HAL_ESP32_CAN1_TX_PIN GPIO_NUM_47
#endif

#ifndef HAL_ESP32_CAN1_RX_PIN
#define HAL_ESP32_CAN1_RX_PIN GPIO_NUM_38
#endif

namespace ESP32 {

// ========== Constructor ==========

CANIface::CANIface(uint8_t index)
    : self_index_(index)
    , initialized_(false)
    , busoff_detected_(false)
    , rx_bytebuffer_(sizeof(CanRxItem) * HAL_CAN_RX_QUEUE_SIZE)
    , rx_queue_(rx_buffer_, HAL_CAN_RX_QUEUE_SIZE)
    , tx_queue_head_(0)
    , tx_queue_tail_(0)
    , event_sem_(nullptr)
    , last_bus_recovery_us_(0)
    , last_error_poll_us_(0)
    , acceptance_code_(0)
    , acceptance_mask_(0xFFFFFFFF)
    , filters_configured_(false)
    , tx_pin_(HAL_ESP32_CAN1_TX_PIN)
    , rx_pin_(HAL_ESP32_CAN1_RX_PIN)
{
    memset(&stats_, 0, sizeof(stats_));
}

// ========== Initialization ==========

bool CANIface::init(const uint32_t bitrate, const OperatingMode mode)
{
    if (initialized_) {
        return true;
    }

    // Store mode
    mode_ = mode;
    bitrate_ = bitrate;

    /*
     * ArduRemoteID 优化策略:
     * 默认使用优先级过滤，只接收 PRIORITY_MEDIUM 或更低优先级的消息
     * 这可以防止 ESP32 在繁忙的 CAN 总线上 CPU 过载
     */
    if (!filters_configured_) {
        // 优先级位过滤: 0x10000000 << 3
        // 只接收优先级 >= 16 的消息
        acceptance_code_ = 0x10000000U << 3;
        acceptance_mask_ = 0x0FFFFFFFU << 3;
    }

    // Initialize TWAI driver
    if (!initTWAI(bitrate, acceptance_code_, acceptance_mask_)) {
        hal.console->printf("CAN%d: TWAI init failed\n", self_index_);
        return false;
    }

    // Setup TWAI alerts
    setupTWAIAlerts();

    // Start TWAI driver
    esp_err_t err = twai_start();
    if (err != ESP_OK) {
        hal.console->printf("CAN%d: TWAI start failed: %d\n", self_index_, err);
        return false;
    }

    initialized_ = true;
    hal.console->printf("CAN%d: Initialized at %lu bps\n", self_index_, bitrate);

    return true;
}

bool CANIface::initTWAI(uint32_t bitrate, uint32_t acceptance_code, uint32_t acceptance_mask)
{
    // Compute bit timings
    Timings timings;
    if (!computeTimings(bitrate, timings)) {
        hal.console->printf("CAN%d: Failed to compute timings for %lu bps\n",
                           self_index_, bitrate);
        return false;
    }

    // TWAI timing configuration
    twai_timing_config_t t_config = {};
    if (bitrate == 1000000) {
        t_config = TWAI_TIMING_CONFIG_1MBITS();
    } else if (bitrate == 500000) {
        t_config = TWAI_TIMING_CONFIG_500KBITS();
    } else if (bitrate == 250000) {
        t_config = TWAI_TIMING_CONFIG_250KBITS();
    } else if (bitrate == 125000) {
        t_config = TWAI_TIMING_CONFIG_125KBITS();
    } else {
        // Custom timing
        t_config.brp = timings.prescaler;
        t_config.tseg_1 = timings.bs1;
        t_config.tseg_2 = timings.bs2;
        t_config.sjw = timings.sjw;
        t_config.triple_sampling = false;
    }

    // TWAI filter configuration
    twai_filter_config_t f_config = {
        .acceptance_code = acceptance_code,
        .acceptance_mask = acceptance_mask,
        .single_filter = true
    };

    // TWAI general configuration (from ArduRemoteID)
    twai_mode_t twai_mode = TWAI_MODE_NORMAL;
    switch (mode_) {
        case NormalMode:
            twai_mode = TWAI_MODE_NORMAL;
            break;
        case SilentMode:
            twai_mode = TWAI_MODE_LISTEN_ONLY;
            break;
        default:
            twai_mode = TWAI_MODE_NORMAL;
            break;
    }

    twai_general_config_t g_config = {
        .mode = twai_mode,
        .tx_io = tx_pin_,
        .rx_io = rx_pin_,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 5,              // ArduRemoteID 设置
        .rx_queue_len = 50,             // ArduRemoteID 设置 (关键!)
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0,
        .intr_flags = ESP_INTR_FLAG_LEVEL2
    };

    // Install TWAI driver
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        hal.console->printf("CAN%d: twai_driver_install failed: %d\n", self_index_, err);
        return false;
    }

    hal.console->printf("CAN%d: TWAI driver installed\n", self_index_);
    return true;
}

void CANIface::setupTWAIAlerts()
{
    // ArduRemoteID 策略: 只启用接收相关告警
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA |
                                TWAI_ALERT_RX_QUEUE_FULL |
                                TWAI_ALERT_BUS_ERROR |
                                TWAI_ALERT_ERR_PASS |
                                TWAI_ALERT_BUS_OFF;

    esp_err_t err = twai_reconfigure_alerts(alerts_to_enable, NULL);
    if (err == ESP_OK) {
        hal.console->printf("CAN%d: TWAI alerts configured\n", self_index_);
    } else {
        hal.console->printf("CAN%d: Failed to configure alerts: %d\n", self_index_, err);
    }
}

// ========== Bit Timing Computation (from ArduRemoteID) ==========

bool CANIface::computeTimings(uint32_t target_bitrate, Timings& out_timings)
{
    if (target_bitrate < 1) {
        return false;
    }

    // ESP32 APB clock frequency (80 MHz for ESP32, 80 MHz for ESP32-S3)
    const uint32_t pclk = 80000000 / 2;  // 40 MHz after prescaler /2

    static const int MaxBS1 = 16;
    static const int MaxBS2 = 8;

    /*
     * Optimal quanta per bit (from ArduRemoteID):
     *   Bitrate        Optimal Maximum
     *   1000 kbps      8       10
     *   500  kbps      16      17
     *   250  kbps      16      17
     *   125  kbps      16      17
     */
    const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;

    static const int MaxSamplePointLocation = 900;  // 90%

    // Computing (prescaler * BS):
    // BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))
    const uint32_t prescaler_bs = pclk / target_bitrate;

    // Search for highest quanta per bit
    uint8_t bs1_bs2_sum = uint8_t(max_quanta_per_bit - 1);

    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
        if (bs1_bs2_sum <= 2) {
            return false;  // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U)) {
        return false;
    }

    // Compute BS1 and BS2 for optimal sample point (87.5%)
    // bs1 = (7 * bs1_bs2_sum - 1) / 8
    // bs2 = bs1_bs2_sum - bs1

    struct BsPair {
        uint8_t bs1;
        uint8_t bs2;
        uint16_t sample_point_permill;

        BsPair(uint8_t bs1_bs2_sum, uint8_t arg_bs1)
            : bs1(arg_bs1)
            , bs2(uint8_t(bs1_bs2_sum - bs1))
            , sample_point_permill(uint16_t(1000 * (1 + bs1) / (1 + bs1 + bs2)))
        {}

        bool isValid() const {
            return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);
        }
    };

    // First attempt with rounding to nearest
    BsPair solution(bs1_bs2_sum, uint8_t(((7 * bs1_bs2_sum - 1) + 4) / 8));

    if (solution.sample_point_permill > MaxSamplePointLocation) {
        // Second attempt with rounding to zero
        solution = BsPair(bs1_bs2_sum, uint8_t((7 * bs1_bs2_sum - 1) / 8));
    }

    if ((target_bitrate != (pclk / (prescaler * (1 + solution.bs1 + solution.bs2)))) ||
        !solution.isValid()) {
        return false;
    }

    hal.console->printf("CAN%d: Timings - quanta/bit: %d, sample point: %.1f%%\n",
                       self_index_,
                       int(1 + solution.bs1 + solution.bs2),
                       float(solution.sample_point_permill) / 10.0f);

    out_timings.prescaler = uint16_t(prescaler - 1U);
    out_timings.sjw = 0;  // Which means 1
    out_timings.bs1 = uint8_t(solution.bs1 - 1);
    out_timings.bs2 = uint8_t(solution.bs2 - 1);

    return true;
}

}  // namespace ESP32

#endif  // HAL_NUM_CAN_IFACES
```

由于篇幅限制，我将继续创建完整的实现方案文档...

