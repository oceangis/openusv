# VSCode + ESP-IDF 项目配置完成

本文档说明 ardupilot_rover_esp32s3_idf 项目已配置为支持 VSCode + ESP-IDF 开发环境。

## 已添加的文件

### 1. 根目录文件

- **CMakeLists.txt** - ESP-IDF 主 CMake 配置文件
  - 来源: 从 `libraries/AP_HAL_ESP32/targets/esp32s3/esp-idf/CMakeLists.txt` 复制
  - 用途: 定义项目构建规则和依赖

- **sdkconfig.defaults** - ESP-IDF 默认配置
  - 来源: 从 `libraries/AP_HAL_ESP32/targets/esp32s3/esp-idf/sdkconfig.defaults` 复制
  - 用途: ESP32-S3 的默认 SDK 配置
  - 主要配置:
    - 目标芯片: ESP32-S3
    - Flash 大小: 8MB
    - CPU 频率: 240MHz
    - FreeRTOS 频率: 1000Hz

- **partitions.csv** - 分区表
  - 来源: 从 `libraries/AP_HAL_ESP32/targets/esp32s3/partitions.csv` 复制
  - 用途: 定义 Flash 分区布局

- **idf_component.yml** - ESP-IDF 组件依赖
  - 新创建
  - 用途: 定义项目依赖的 ESP-IDF 组件

- **BUILD_IDF.md** - 编译说明文档
  - 新创建
  - 用途: 详细的编译和使用指南

### 2. main 目录

- **main/CMakeLists.txt** - main 组件配置
  - 新创建
  - 用途: 定义 main 组件的源文件和包含目录

- **main/main.c** - 程序入口
  - 来源: 从 `libraries/AP_HAL_ESP32/targets/esp32s3/esp-idf/main.c` 复制
  - 用途: ESP-IDF 程序入口，调用 ArduPilot 的 main() 函数

### 3. .vscode 目录配置

- **.vscode/c_cpp_properties.json** - C/C++ IntelliSense 配置
  - 新创建
  - 用途: 配置 VSCode C/C++ 扩展的 IntelliSense

- **.vscode/settings.json** - VSCode 项目设置
  - 新创建
  - 用途: ESP-IDF 扩展配置，包括目标芯片、端口等

- **.vscode/launch.json** - 调试配置
  - 新创建
  - 用途: ESP-IDF 调试器配置

## 项目结构

```
ardupilot_rover_esp32s3_idf/
├── CMakeLists.txt              # ✓ ESP-IDF 主配置
├── sdkconfig.defaults          # ✓ SDK 默认配置
├── partitions.csv              # ✓ 分区表
├── idf_component.yml           # ✓ 组件依赖
├── BUILD_IDF.md                # ✓ 编译说明
│
├── main/                       # ✓ 主程序目录
│   ├── CMakeLists.txt         # ✓ main 组件配置
│   └── main.c                 # ✓ 入口文件
│
├── .vscode/                    # ✓ VSCode 配置
│   ├── c_cpp_properties.json  # ✓ IntelliSense 配置
│   ├── settings.json          # ✓ ESP-IDF 扩展配置
│   └── launch.json            # ✓ 调试配置
│
├── libraries/                  # ArduPilot 库
│   ├── AP_HAL_ESP32/          # ESP32 HAL 实现
│   │   └── targets/
│   │       └── esp32s3/
│   │           └── esp-idf/   # 原始 ESP-IDF 配置 (保留)
│   ├── AP_Common/
│   ├── AP_Math/
│   └── ... (其他库)
│
├── Rover/                      # Rover 主程序
├── Tools/                      # 构建工具
├── waf                         # WAF 构建脚本
└── wscript                     # WAF 配置
```

## 使用方法

### 快速开始

1. **打开项目**
   ```
   用 VSCode 打开 ardupilot_rover_esp32s3_idf 文件夹
   ```

2. **安装 ESP-IDF 扩展**
   - 扩展 ID: `espressif.esp-idf-extension`

3. **配置 ESP-IDF**
   - 按 `F1` -> `ESP-IDF: Configure ESP-IDF Extension`
   - 选择已安装的 ESP-IDF v5.3+ 版本

4. **设置目标**
   - 按 `F1` -> `ESP-IDF: Set Espressif Device Target`
   - 选择 `esp32s3`

5. **编译**
   - 按 `F1` -> `ESP-IDF: Build your Project`
   - 或点击底部状态栏的 Build 按钮

详细使用说明请参考 [BUILD_IDF.md](BUILD_IDF.md)

## 重要说明

### 当前使用 WAF 构建系统

**注意**: ArduPilot 当前仍使用 WAF 构建系统，而非纯 ESP-IDF CMake 系统。

添加的 ESP-IDF 文件主要用于:
1. ✅ VSCode IntelliSense 支持
2. ✅ 代码导航和自动完成
3. ✅ 语法高亮和错误检测
4. ✅ ESP-IDF 扩展集成
5. 🔄 为未来完全迁移到 ESP-IDF CMake 做准备

### 推荐的编译方法

**当前推荐使用 WAF**:
```bash
# 配置
./waf configure --board=esp32s3devkit

# 编译
./waf rover

# 烧录
./waf rover --upload
```

## 特性支持

### ✅ 已支持

- ESP32-S3 开发
- VSCode 智能感知
- 代码导航
- 语法检查
- ESP-IDF 扩展集成
- JTAG 调试配置
- 串口监控

### 🔄 部分支持

- ESP-IDF CMake 构建 (配置已就绪，但需要适配)
- VSCode 一键编译 (需要使用 WAF)

### ❌ 已移除

- ChibiOS 支持 (100% 清理)
- Linux 支持 (100% 清理)
- QURT 支持 (100% 清理)

## 配置文件说明

### CMakeLists.txt

- 定义项目名称: `ardupilot`
- 设置工具链: `toolchain-esp32s3.cmake`
- 配置 ESP-IDF 组件
- 链接 ArduPilot 库

### sdkconfig.defaults

关键配置项:
- `CONFIG_IDF_TARGET="esp32s3"` - 目标芯片
- `CONFIG_ESPTOOLPY_FLASHSIZE_8MB` - 8MB Flash
- `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240` - 240MHz CPU
- `CONFIG_FREERTOS_HZ=1000` - 1kHz 调度器
- `CONFIG_COMPILER_OPTIMIZATION_PERF` - 性能优化

### .vscode/settings.json

- `idf.adapterTargetName`: "esp32s3" - 目标芯片
- `idf.flashType`: "UART" - 烧录方式
- `idf.openOcdConfigs`: JTAG 调试配置

## 下一步

1. **测试编译**
   ```bash
   cd ardupilot_rover_esp32s3_idf
   ./waf configure --board=esp32s3devkit
   ./waf rover
   ```

2. **配置串口**
   - 在 `.vscode/settings.json` 中设置 `idf.port`
   - Windows: "COM3"
   - Linux: "/dev/ttyUSB0"

3. **烧录测试**
   ```bash
   ./waf rover --upload
   ```

4. **监控串口**
   ```bash
   idf.py monitor
   ```

## 故障排除

### VSCode IntelliSense 不工作

1. 按 `F1` -> `C/C++: Edit Configurations (UI)`
2. 检查 `compile_commands.json` 路径
3. 重新加载窗口: `F1` -> `Reload Window`

### ESP-IDF 扩展找不到 IDF_PATH

1. 按 `F1` -> `ESP-IDF: Configure ESP-IDF Extension`
2. 手动选择 ESP-IDF 安装路径
3. 例如: `D:\Espressif\v5.5.1\esp-idf`

### 编译错误

1. 确保使用 WAF 而非 ESP-IDF CMake
2. 清理构建: `./waf clean`
3. 重新配置: `./waf configure --board=esp32s3devkit`

## 参考文档

- [BUILD_IDF.md](BUILD_IDF.md) - 详细编译说明
- [ESP32_README.md](ESP32_README.md) - ESP32 特定说明
- [README_IDF.md](README_IDF.md) - IDF 版本说明

## 维护者

本配置由 Claude Code 自动生成和配置。

## 更新日志

### 2024-10-24
- ✅ 创建 ESP-IDF 项目结构
- ✅ 配置 VSCode 集成
- ✅ 添加编译文档
- ✅ 100% 清理 ChibiOS/Linux/QURT 代码
