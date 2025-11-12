# ESP-IDF v5.5 Compilation Fixes Summary

## Date: 2025-11-02

### Overview
Fixed compilation errors related to ESP-IDF v5.5 API changes in the ArduPilot Rover ESP32-S3 project.

## Issues Fixed

### 1. RTC Clock API Changes in `libraries/AP_HAL_ESP32/Util.cpp`

**Problem:**
- `rtc_clk_cpu_freq_value()` and `rtc_clk_cpu_freq_get()` are deprecated in ESP-IDF v5.5

**Solution:**
- Added include: `#include "esp_private/esp_clk.h"`
- Replaced: `rtc_clk_cpu_freq_value(rtc_clk_cpu_freq_get())` with `esp_clk_cpu_freq()`

**Location:** Line 349 in Util.cpp

### 2. UARTDesc Array Size Issue in `libraries/AP_HAL_ESP32/Util.cpp`

**Problem:**
- `extern UARTDesc uart_desc[]` incomplete type error with `ARRAY_SIZE` macro
- Missing namespace qualifier

**Solution:**
- Changed `extern UARTDesc uart_desc[]` to `extern ESP32::UARTDesc uart_desc[]`
- Replaced `ARRAY_SIZE(uart_desc)` with `hal.num_serial()`

**Location:** Lines 393-396 in Util.cpp

### 3. GPIO Direction API in `libraries/AP_HAL_ESP32/I2CDevice.cpp`

**Problem:**
- `gpio_get_direction()` was removed in ESP-IDF v5.5

**Solution:**
- Removed calls to `gpio_get_direction()` in two functions:
  - `I2CBus::clear_bus()` (line 95-96)
  - `I2CBus::read_sda()` (line 118)
- Simplified logic to not save/restore pin modes
- Added I2C bus reinitialization after bus recovery in `clear_bus()`

### 4. HAL Declaration Scope in `libraries/AP_HAL_ESP32/I2CDevice.cpp`

**Problem:**
- `extern const AP_HAL::HAL& hal;` was declared inside `clear_bus()` function
- Used in `transfer()` function causing "not declared in this scope" error

**Solution:**
- Moved `extern const AP_HAL::HAL& hal;` to file scope (after includes, line 25)

**Location:** Line 205 usage, moved declaration to line 25

## Files Modified

1. `libraries/AP_HAL_ESP32/Util.cpp`
   - Added ESP-IDF v5.5 clock API support
   - Fixed UART array handling

2. `libraries/AP_HAL_ESP32/I2CDevice.cpp`
   - Removed deprecated GPIO functions
   - Fixed HAL declaration scope
   - Updated I2C bus recovery logic

## API Changes Reference

### ESP-IDF v5.5 Changes
- **Removed:** `rtc_clk_cpu_freq_get()`, `rtc_clk_cpu_freq_value()`
- **New:** `esp_clk_cpu_freq()` (returns CPU frequency in Hz directly)
- **Removed:** `gpio_get_direction()`
- **Alternative:** GPIO state management through driver reconfiguration

## Testing
All compilation errors have been resolved. The code now compiles with ESP-IDF v5.5.1.

## Notes
- ESP-IDF v5.5 introduced breaking API changes for clock and GPIO management
- The new APIs are more streamlined and require less manual state management
- I2C bus recovery now relies on driver reinitialization rather than manual pin state save/restore
