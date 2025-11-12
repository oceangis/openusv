# ESP32-S3 HAL快速参考

## 🚀 5分钟上手指南

---

## 1️⃣ Monitor线程（自动启用）

```cpp
// ✅ 自动工作，无需配置
// 主循环延迟时会自动打印警告
```

**输出示例**:
```
WARNING: Main loop delay 250ms
CRITICAL: Main loop stuck 600ms!
```

---

## 2️⃣ 合法长操作

```cpp
// SD卡初始化、Flash写入等
hal.scheduler->expect_delay_ms(2000);  // 开始
mount_sdcard();                        // 执行
hal.scheduler->expect_delay_ms(0);     // 结束
```

---

## 3️⃣ I2C批量读取

```cpp
// 一次读取14字节（加速度+陀螺仪+温度）
uint8_t data[14];
dev->read_registers_multiple(0x3B, data, 14, 1);
```

---

## 4️⃣ GPIO中断

```cpp
// 定义回调（在ISR中运行，必须快！）
void my_isr(uint8_t pin, bool state, uint32_t ts) {
    data_ready = true;  // 只设置标志
}

// 绑定中断
hal.gpio->attach_interrupt(15, my_isr, INTERRUPT_RISING);

// 解绑（可选）
hal.gpio->detach_interrupt(15);
```

**触发模式**:
- `INTERRUPT_RISING` - 上升沿
- `INTERRUPT_FALLING` - 下降沿
- `INTERRUPT_BOTH` - 双边沿
- `INTERRUPT_LOW` - 低电平
- `INTERRUPT_HIGH` - 高电平

---

## 5️⃣ CAN过滤（自动优化）

```cpp
// ✅ 自动工作
// 热门消息会自动移到前面
// 每1000次命中检查并优化
```

---

## ⚠️ 重要提示

### ISR回调规则
```cpp
void isr_callback(uint8_t pin, bool state, uint32_t timestamp) {
    // ✅ 可以：
    flag = true;           // 设置标志
    count++;               // 计数

    // ❌ 不能：
    delay(10);             // 延迟
    printf("hi");          // 打印
    i2c->read();           // I2C/SPI
    malloc();              // 动态内存
}
```

### 典型应用

**IMU数据就绪**:
```cpp
#define IMU_INT 15
volatile bool imu_ready = false;

void imu_isr(uint8_t pin, bool state, uint32_t ts) {
    imu_ready = true;
}

void setup() {
    hal.gpio->attach_interrupt(IMU_INT, imu_isr, INTERRUPT_RISING);
}

void loop() {
    if (imu_ready) {
        imu_ready = false;
        read_imu();
    }
}
```

**GPS PPS同步**:
```cpp
#define GPS_PPS 16
volatile uint64_t pps_time = 0;

void pps_isr(uint8_t pin, bool state, uint32_t ts) {
    pps_time = AP_HAL::micros64();
}

void setup() {
    hal.gpio->attach_interrupt(GPS_PPS, pps_isr, INTERRUPT_RISING);
}
```

**脉冲计数**:
```cpp
#define FLOW_PIN 17
volatile uint32_t pulses = 0;

void flow_isr(uint8_t pin, bool state, uint32_t ts) {
    pulses++;
}

void setup() {
    hal.gpio->attach_interrupt(FLOW_PIN, flow_isr, INTERRUPT_BOTH);
}
```

---

## 📝 快速检查清单

启动时应该看到:
- ✅ `OK created task APM_MONITOR on SLOWCPU`
- ✅ `Monitor thread started`

如果主循环卡住:
- ✅ `WARNING: Main loop delay XXXms`
- ✅ `CRITICAL: Main loop stuck XXXms!`

GPIO中断测试:
```cpp
// 测试代码
volatile uint32_t count = 0;
void test_isr(uint8_t pin, bool state, uint32_t ts) { count++; }

void test() {
    hal.gpio->attach_interrupt(15, test_isr, INTERRUPT_BOTH);
    // 手动触发引脚15几次
    printf("Count: %lu\n", count);  // 应该增加
}
```

---

## 🎯 常见问题

**Q: Monitor线程如何工作？**
A: 自动检查主循环延迟，无需配置。

**Q: GPIO中断最多支持多少个？**
A: ESP32-S3所有GPIO都支持（GPIO0-47）。

**Q: 中断回调可以打印吗？**
A: ❌ 不能！ISR中只能做简单操作。

**Q: 如何调试中断不触发？**
A: 添加计数器，检查引脚配置和触发模式。

**Q: CAN过滤器需要手动优化吗？**
A: ❌ 不需要！自动优化。

---

## 📚 详细文档

- `GPIO_INTERRUPT_EXAMPLE.md` - GPIO中断详细示例
- `ESP32_HAL_IMPROVEMENTS.md` - 完整改进说明

---

**快速开始，保持简单！** 🚀
