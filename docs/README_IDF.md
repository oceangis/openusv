# ArduPilot Rover ESP32-S3 纯 IDF 版本

这是 ArduPilot 的纯 ESP-IDF 版本，专门为 ESP32/ESP32-S3 和 SITL 平台优化。

## 🎯 项目特点

- ✅ **仅支持 ESP32/ESP32-S3 和 SITL**：移除了 ChibiOS、Linux、QURT 等其他平台代码
- ✅ **纯 IDF 构建系统**：完全基于 ESP-IDF 和 waf 构建
- ✅ **精简高效**：删除了不必要的平台支持代码
- ✅ **保留原始文件结构**：目录结构与原始 ArduPilot 保持一致

## 📦 已删除的内容

### 平台支持
- ChibiOS 操作系统及相关代码
- Linux 平台支持
- QURT 平台支持

### 具体删除的文件
1. **ChibiOS 相关**
   - `libraries/AP_MultiHeap/MultiHeap_chibios.cpp`
   - `libraries/AP_Networking/AP_Networking_ChibiOS.cpp`
   - `libraries/AP_Networking/AP_Networking_ChibiOS.h`
   - `.github/workflows/test_chibios.yml`

2. **Linux 相关**
   - `libraries/AP_HAL/utility/RCOutput_Tap_Linux.cpp`
   - `.github/workflows/test_linux_sbc.yml`

3. **构建配置**
   - `Tools/ardupilotwaf/boards.py` 中的 `chibios` 类定义（251行）
   - `Tools/ardupilotwaf/boards.py` 中的 `linux` 类定义（122行）
   - `Tools/ardupilotwaf/boards.py` 中的 `QURT` 类定义（95行）
   - ChibiOS hwdef 路径引用

## 📂 项目结构

```
ardupilot_rover_esp32s3_idf/
├── libraries/              # 核心库
│   ├── AP_HAL_ESP32/      # ESP32 HAL 实现
│   ├── AP_HAL_SITL/       # SITL HAL 实现
│   └── ...                # 其他通用库
├── Rover/                 # Rover 固件源码
├── Tools/                 # 构建工具
│   └── ardupilotwaf/      # WAF 构建系统
│       └── boards.py      # 仅包含 ESP32 和 SITL 板级定义
├── modules/               # 子模块（需单独获取）
│   └── esp_idf/          # ESP-IDF
└── build/                 # 构建输出
```

## 🚀 快速开始

### 1. 前提条件

```bash
# 确保已安装 ESP-IDF
# 本项目基于 ESP-IDF v5.5
```

### 2. 获取依赖模块

```bash
# 初始化子模块
git submodule update --init --recursive

# 或者只获取必需的模块
git submodule update --init modules/esp_idf
```

### 3. 配置 ESP-IDF 环境

```bash
# 进入 esp_idf 目录并安装
cd modules/esp_idf
./install.sh esp32s3

# 导出环境变量
. export.sh
cd ../..
```

### 4. 配置编译

```bash
# 配置 ESP32-S3 板子
./waf configure --board=esp32s3devkit

# 可用的 ESP32 板子：
# - esp32s3devkit
# - esp32s3empty
# - esp32buzz
# - esp32diy
```

### 5. 编译 Rover 固件

```bash
# 编译 Rover
./waf rover

# 固件输出位置：
# build/esp32s3devkit/esp-idf_build/ardupilot.bin
```

### 6. 烧录固件

```bash
# 自动烧录
./waf rover --upload

# 或手动烧录
esptool.py --chip esp32s3 --port COM3 write_flash 0x0 build/esp32s3devkit/esp-idf_build/ardupilot.bin
```

## 🔧 SITL 仿真

```bash
# 配置 SITL
./waf configure --board=sitl

# 编译 SITL
./waf rover

# 运行 SITL
./build/sitl/bin/ardurover --model rover
```

## 📊 代码统计

### boards.py 文件变化
- **原始行数**: 1725 行
- **删除后行数**: 1339 行
- **删除行数**: 386 行
- **精简比例**: 22.4%

### 删除的类定义
- `chibios` 类: ~251 行
- `linux` 类: ~122 行
- `QURT` 类: ~95 行
- 其他清理: ~18 行

### 保留的板级类
- `Board` - 基础板级类
- `sitl` - SITL 仿真平台
- `esp32` - ESP32 平台基类
- `esp32s3` - ESP32-S3 具体实现

## 🔍 技术细节

### 支持的平台
1. **ESP32/ESP32-S3**
   - HAL: `AP_HAL_ESP32`
   - 板级定义: `HAL_BOARD_ESP32`
   - 构建系统: ESP-IDF + waf

2. **SITL (Software In The Loop)**
   - HAL: `AP_HAL_SITL`
   - 板级定义: `HAL_BOARD_SITL`
   - 用于仿真和测试

### 构建系统
- 主构建脚本: `wscript`
- 板级配置: `Tools/ardupilotwaf/boards.py`
- 平台检测: 基于 `HAL_BOARD_*` 宏定义

### HAL 层次结构
```
AP_HAL (抽象接口)
├── AP_HAL_ESP32 (ESP32 实现)
└── AP_HAL_SITL (SITL 实现)
```

## 📝 修改记录

### v1.0 - 纯 IDF 版本
- ✅ 删除 ChibiOS 平台支持（6个文件）
- ✅ 删除 Linux 平台支持（2个文件）
- ✅ 删除 QURT 平台支持（构建配置）
- ✅ 清理 boards.py，移除 3 个板级类定义（386行）
- ✅ 保留原始文件结构
- ✅ 仅支持 ESP32/ESP32-S3 和 SITL

## 🔗 相关链接

- [ArduPilot 官方](https://ardupilot.org/)
- [ArduPilot ESP32 文档](libraries/AP_HAL_ESP32/README.md)
- [ESP-IDF 文档](https://docs.espressif.com/projects/esp-idf/)
- [ESP32-S3 数据手册](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)

## 📄 许可证

ArduPilot 采用 GPLv3 许可证。详见 [COPYING.txt](COPYING.txt)

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

---

**维护者**: Claude Code
**最后更新**: 2025-10-24
**基于**: oceangis/ardupilot_esp32
