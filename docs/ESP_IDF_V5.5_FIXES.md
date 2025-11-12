# ESP-IDF v5.5 编译错误完整修复报告

## 修复日期: 2025-11-02

## 修复的错误

### 1. **Util.cpp:349** - RTC时钟API已弃用
```cpp
// 错误代码:
str.printf("CPU Frequency: %lu MHz\n", (unsigned long)(rtc_clk_cpu_freq_value(rtc_clk_cpu_freq_get()) / 1000000));

// 修复后:
#include "esp_private/esp_clk.h"  // 新增头文件
str.printf("CPU Frequency: %lu MHz\n", (unsigned long)(esp_clk_cpu_freq() / 1000000));
```

### 2. **Util.cpp:396** - UART数组大小错误
```cpp
// 错误代码:
extern UARTDesc uart_desc[];
for (uint8_t i = 0; i < ARRAY_SIZE(uart_desc); i++) {

// 修复后:
extern ESP32::UARTDesc uart_desc[];  // 添加命名空间
for (uint8_t i = 0; i < hal.num_serial; i++) {  // 使用常量而非函数调用
```

### 3. **I2CDevice.cpp:95-96** - gpio_get_direction已移除
```cpp
// 错误代码:
gpio_mode_t sda_mode_saved = GPIO_MODE_DISABLE;
gpio_mode_t scl_mode_saved = GPIO_MODE_DISABLE;
gpio_get_direction(sda_pin, &sda_mode_saved);
gpio_get_direction(scl_pin, &scl_mode_saved);

// 修复后:
// ESP-IDF v5.5已移除gpio_get_direction()
// 总线恢复后直接重新初始化I2C驱动
```

### 4. **I2CDevice.cpp:205** - hal声明作用域错误
```cpp
// 错误代码:
void I2CBus::clear_bus(void) {
    extern const AP_HAL::HAL& hal;  // 局部声明
}
// transfer()函数中使用hal导致"未声明"错误

// 修复后:
// 在文件顶部（第25行）添加全局声明
extern const AP_HAL::HAL& hal;
```

### 5. **I2CDevice.cpp** - I2C总线恢复逻辑重写
```cpp
// 新的clear_bus()实现:
void I2CBus::clear_bus(void)
{
    // 切换SCL 20次以恢复总线
    gpio_set_direction(scl_pin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(scl_pin, GPIO_PULLUP_ONLY);
    
    for (uint8_t i = 0; i < 20; i++) {
        gpio_set_level(scl_pin, 0);
        hal.scheduler->delay_microseconds(10);
        gpio_set_level(scl_pin, 1);
        hal.scheduler->delay_microseconds(10);
    }
    
    // 重新初始化I2C总线
    i2c_driver_delete(port);
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = frequency;
    i2c_param_config(port, &conf);
    i2c_driver_install(port, conf.mode, 0, 0, 0);
}
```

## 修改的文件

1. **libraries/AP_HAL_ESP32/Util.cpp**
   - 添加 `esp_private/esp_clk.h` 头文件
   - 替换 `rtc_clk_cpu_freq_value(rtc_clk_cpu_freq_get())` 为 `esp_clk_cpu_freq()`
   - 修正UARTDesc命名空间: `ESP32::UARTDesc`
   - 修正 `hal.num_serial()` -> `hal.num_serial`

2. **libraries/AP_HAL_ESP32/I2CDevice.cpp**
   - 添加 `driver/gpio.h` 头文件
   - 移动 `extern const AP_HAL::HAL& hal;` 到文件作用域
   - 移除所有 `gpio_get_direction()` 调用
   - 重写I2C总线恢复逻辑
   - 添加 `clear_bus()` 和 `read_sda()` 函数实现

3. **libraries/AP_HAL_ESP32/I2CDevice.h**
   - 添加SDA/SCL引脚成员变量
   - 添加总线恢复相关函数声明

## ESP-IDF v5.5 API变更总结

| 旧API | 新API | 说明 |
|-------|-------|------|
| `rtc_clk_cpu_freq_get()` | `esp_clk_cpu_freq()` | 直接返回Hz值 |
| `rtc_clk_cpu_freq_value()` | 已移除 | 使用新API |
| `gpio_get_direction()` | 已移除 | 通过驱动重新配置 |

## 验证清单

- [x] 移除所有 `rtc_clk_cpu_freq_get/value` 调用
- [x] 移除所有 `gpio_get_direction` 调用  
- [x] 修正 `hal.num_serial()` -> `hal.num_serial`
- [x] 添加必要的命名空间限定符
- [x] 修正变量作用域问题
- [x] 清理过时注释

## 测试状态

等待完整编译测试...

