# ArduPilot Rover ESP32-S3 IDF 编译指南

本文档说明如何使用 VSCode + ESP-IDF 编译 ArduPilot Rover for ESP32-S3。

## 前提条件

1. **安装 ESP-IDF v5.3 或更高版本**
   - Windows: 使用 ESP-IDF Tools Installer
   - 下载: https://dl.espressif.com/dl/esp-idf/

2. **安装 VSCode 扩展**
   - ESP-IDF Extension (espressif.esp-idf-extension)

3. **硬件要求**
   - ESP32-S3 开发板 (推荐 N16R8: 16MB Flash, 8MB PSRAM)
   - USB 数据线

## 项目结构

```
ardupilot_rover_esp32s3_idf/
├── CMakeLists.txt              # 主 CMake 配置文件
├── sdkconfig.defaults          # ESP-IDF 默认配置
├── partitions.csv              # 分区表
├── idf_component.yml           # 组件依赖
├── main/                       # 主程序目录
│   ├── CMakeLists.txt
│   └── main.c                  # 入口文件
├── libraries/                  # ArduPilot 库
│   ├── AP_HAL_ESP32/          # ESP32 HAL 实现
│   ├── AP_Common/
│   ├── AP_Math/
│   └── ... (其他库)
├── Rover/                      # Rover 主程序
└── .vscode/                    # VSCode 配置
    ├── c_cpp_properties.json
    ├── settings.json
    └── launch.json
```

## 编译步骤

### 方法 1: 使用 VSCode ESP-IDF 扩展 (推荐)

1. **打开项目**
   ```
   用 VSCode 打开 ardupilot_rover_esp32s3_idf 文件夹
   ```

2. **配置 IDF 路径**
   - 按 `F1` 或 `Ctrl+Shift+P`
   - 选择 `ESP-IDF: Configure ESP-IDF Extension`
   - 选择已安装的 ESP-IDF 版本

3. **设置目标芯片**
   - 按 `F1`
   - 选择 `ESP-IDF: Set Espressif Device Target`
   - 选择 `esp32s3`

4. **编译项目**
   - 按 `F1`
   - 选择 `ESP-IDF: Build your Project`
   - 或点击底部状态栏的 "Build" 图标

5. **烧录固件**
   - 连接 ESP32-S3 开发板
   - 按 `F1`
   - 选择 `ESP-IDF: Flash your Project`
   - 或点击底部状态栏的 "Flash" 图标

6. **监控串口**
   - 按 `F1`
   - 选择 `ESP-IDF: Monitor your Device`
   - 或点击底部状态栏的 "Monitor" 图标

### 方法 2: 使用命令行

1. **初始化 ESP-IDF 环境**
   ```bash
   # Windows (使用 ESP-IDF Command Prompt)
   # 或在普通 cmd 中运行:
   D:\Espressif\v5.5.1\esp-idf\export.bat

   # Linux/Mac
   . $HOME/esp/esp-idf/export.sh
   ```

2. **设置目标芯片**
   ```bash
   cd ardupilot_rover_esp32s3_idf
   idf.py set-target esp32s3
   ```

3. **配置项目 (可选)**
   ```bash
   idf.py menuconfig
   ```

4. **编译**
   ```bash
   idf.py build
   ```

5. **烧录**
   ```bash
   # 替换 COMX 为实际串口号
   idf.py -p COMX flash
   ```

6. **监控串口**
   ```bash
   idf.py -p COMX monitor
   ```

7. **一键编译+烧录+监控**
   ```bash
   idf.py -p COMX flash monitor
   ```

## 注意: 当前项目使用 WAF 构建系统

**重要提示**: 当前版本的 ArduPilot 使用 WAF 构建系统，而不是纯 ESP-IDF CMake 系统。

### 使用 WAF 编译 (当前方法)

1. **配置 WAF**
   ```bash
   cd ardupilot_rover_esp32s3_idf
   ./waf configure --board=esp32s3devkit
   ```

2. **编译 Rover**
   ```bash
   ./waf rover
   ```

3. **烧录到 ESP32-S3**
   ```bash
   ./waf rover --upload
   ```

### ESP-IDF 集成说明

本项目提供的 ESP-IDF 文件（CMakeLists.txt, sdkconfig.defaults 等）是为了：
1. 提供 VSCode IntelliSense 支持
2. 方便代码导航和调试
3. 为未来完全迁移到 ESP-IDF CMake 系统做准备

**当前推荐的编译方法仍然是使用 WAF**，ESP-IDF 文件主要用于 IDE 支持。

## 配置说明

### sdkconfig.defaults

主要配置项:
- `CONFIG_IDF_TARGET="esp32s3"` - 目标芯片
- `CONFIG_ESPTOOLPY_FLASHSIZE_8MB` - Flash 大小 8MB
- `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240` - CPU 频率 240MHz
- `CONFIG_FREERTOS_HZ=1000` - FreeRTOS 时钟频率 1000Hz
- `CONFIG_COMPILER_OPTIMIZATION_PERF` - 性能优化

### partitions.csv

分区配置 (8MB Flash):
- nvs: 非易失性存储
- otadata: OTA 数据
- app0/app1: 应用分区 (OTA)
- storage: 用户数据存储

## 常见问题

### 1. 编译错误: "espressif__esp_wifi not found"
```bash
# 更新组件
idf.py update-dependencies
```

### 2. 烧录失败: "Failed to connect"
- 检查 USB 线缆
- 确保驱动已安装
- 尝试按住 BOOT 按钮并重启开发板

### 3. VSCode IntelliSense 不工作
- 按 `F1` -> `C/C++: Edit Configurations (UI)`
- 确保 `compile_commands.json` 路径正确

### 4. 内存不足
- 在 `sdkconfig` 中增加堆大小
- 减少日志级别
- 优化代码

## 调试

### 使用 JTAG 调试

1. **连接 JTAG**
   - 使用 ESP-Prog 或其他 JTAG 适配器
   - 连接 GPIO 19/20 (USB JTAG)

2. **启动调试**
   - 按 `F5` 启动调试
   - 或按 `F1` -> `ESP-IDF: Launch GUI Configuration Tool`

### 使用串口调试

```bash
# 查看日志
idf.py monitor

# 设置日志级别
idf.py menuconfig
# Component config -> Log output -> Default log verbosity
```

## 性能优化

1. **编译器优化**
   - 在 menuconfig 中选择 "Optimize for performance"

2. **Flash 速度**
   - 使用 QIO 模式
   - 提高 Flash 频率到 80MHz

3. **CPU 频率**
   - 设置为 240MHz (已默认)

4. **FreeRTOS 配置**
   - 调整任务优先级
   - 优化堆栈大小

## 支持的功能

- ✅ ESP32-S3 HAL 实现
- ✅ SITL 仿真支持
- ✅ WiFi 连接
- ✅ UART 通信
- ✅ I2C/SPI 外设
- ✅ PWM 输出
- ✅ ADC 读取
- ✅ SD 卡存储
- ✅ 日志记录

## 不支持的功能

- ❌ ChibiOS (已移除)
- ❌ Linux (已移除)
- ❌ QURT (已移除)

## 参考资料

- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/)
- [ArduPilot 开发文档](https://ardupilot.org/dev/)
- [ESP32-S3 技术参考手册](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_cn.pdf)

## 许可证

本项目遵循 ArduPilot 的 GPL v3 许可证。

## 贡献

欢迎提交 Issue 和 Pull Request！
