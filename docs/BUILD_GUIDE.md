# ArduPilot Rover ESP32-S3 编译指南

## 硬件配置

- **主控板**: ESP32-S3 N16R8 (16MB Flash, 8MB PSRAM)
- **IMU**: ICM20948 (9轴, I2C地址0x68)
- **GPS**: u-blox MAX-M10S (UART1, GPIO17/18, 38400波特率)
- **I2C总线**: GPIO20(SDA), GPIO21(SCL)
- **PWM输出**: GPIO11-6 (6通道)
- **CAN接口**: GPIO47(TX), GPIO38(RX) - 暂未启用

## 当前配置状态

- ✅ 8MB PSRAM已启用 (Quad模式, 80MHz)
- ✅ ICM20948 IMU (包含AK09916磁力计)
- ✅ GPS配置 (UART1, 38400波特率)
- ✅ WiFi支持 (SSID: ardupilot123)
- ⏸️ DroneCAN暂时禁用 (缺少libcanard库)

## 快速开始

### 1. 环境准备

确保已安装ESP-IDF 5.5.1环境。build.bat脚本使用以下路径：
- Python环境: `D:\Espressif\tools\python_env\idf5.5_py3.11_env`
- ESP-IDF: `D:\Espressif\v5.5.1\esp-idf`

如果路径不同，请修改build.bat中的配置。

### 2. 修改串口号

编辑 `build.bat`，修改串口号：
```batch
set COM_PORT=COM3  # 改为你的实际串口号
```

### 3. 编译和烧录

**完整流程（推荐首次使用）**:
```batch
build.bat all
```
这会执行：清理 → 编译 → 擦除Flash → 烧录 → 监控

**单独操作**:
```batch
# 清理构建
build.bat clean

# 仅编译
build.bat build

# 擦除Flash（重要！解决启动循环问题）
build.bat erase

# 烧录固件
build.bat flash

# 监控串口输出
build.bat monitor
```

### 4. 查看帮助
```batch
build.bat help
```

## 重要说明

### Flash擦除的必要性

如果遇到系统不断重启的问题，**必须**先擦除Flash：
```batch
build.bat erase
```

原因：ArduPilot的参数存储需要初始化Flash分区，未初始化会导致启动失败。

### 二进制文件位置

编译成功后，固件位于：
```
build/ardupilot_rover_esp32s3.bin
```

## 启用DroneCAN（未来开发）

当前DroneCAN已临时禁用。要启用需要：

### 1. 获取libcanard库

从源ArduPilot仓库初始化submodule：
```bash
cd f:\opensource\usv_esp32\ardupilot-master
git submodule update --init --recursive modules/DroneCAN/libcanard
```

然后复制到项目：
```bash
xcopy /E /I /Y f:\opensource\usv_esp32\ardupilot-master\modules\DroneCAN\libcanard ^
              f:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\modules\DroneCAN\libcanard
```

### 2. 恢复DroneCAN配置

恢复 `libraries/AP_HAL/board/esp32.h`:
```c
#define HAL_WITH_DRONECAN 1
#define HAL_WITH_UAVCAN 1
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 1
#define HAL_NUM_CAN_IFACES 1
```

### 3. 更新hwdef.dat

取消CAN配置注释（`libraries/AP_HAL_ESP32/hwdef/esp32s3_icm20948/hwdef.dat`）：
```python
ESP32_CAN  TWAI_NUM_0  GPIO_NUM_47  GPIO_NUM_38  1000000
define HAL_NUM_CAN_IFACES 1
define HAL_ENABLE_DRONECAN_DRIVERS 1
```

### 4. 重新生成hwdef.h

```bash
cd libraries/AP_HAL_ESP32/hwdef/esp32s3_icm20948
python ../../scripts/esp32_hwdef.py hwdef.dat
```

## 传感器外设配置

根据用户记忆中的配置：

| 编号 | 设备 | 端口 | 波特率 | 供电 | 备注 |
|-----|------|------|--------|------|------|
| 1 | 测深仪dst800 | ESP32-uart2-485 | 4800 | 电池直连 | 独立 |
| 2 | 4G | ch9434-uart0-TTL | 9600 | 电池直连 | 独立 |
| 3 | 海流 | ch9434-uart1-232 | 115200 | 电池直连 | 独立 |
| 4 | 水质仪 | ch9434-uart2-232 | 9600 | 电池直连 | 独立 |
| 5 | 气象 | ch9434-uart3-232 | 4800 | CH_GPIO12控制 | 12V-3电源 |

这些外设需要在后续开发中配置到相应的UART端口。

## 故障排查

### 编译错误

1. **libcanard找不到**: DroneCAN依赖，已临时禁用
2. **Python环境错误**: 检查build.bat中的Python路径
3. **内存不足**: 8MB PSRAM已启用，应该足够

### 烧录失败

1. 检查USB线缆连接
2. 确认COM口号正确
3. 按住BOOT键再按RESET键进入下载模式
4. 检查驱动是否正常（CP2102/CH340等）

### 启动循环

执行Flash擦除：
```batch
build.bat erase
build.bat flash
```

### 串口监视器乱码

检查波特率设置（默认115200）。可在sdkconfig中修改：
```
CONFIG_ESPTOOLPY_MONITOR_BAUD=115200
```

## 日志文件

系统日志路径（如果已配置）：
```
f:\opensource\usv_esp32\esp32s3rover\log\log.txt
```

## 参考文档

- ESP-IDF文档: https://docs.espressif.com/projects/esp-idf/
- ArduPilot文档: https://ardupilot.org/
- ICM20948数据手册: https://invensense.tdk.com/
- u-blox MAX-M10S: https://www.u-blox.com/

## 技术支持

- 项目路径: `f:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf`
- ArduPilot源码: `f:\opensource\usv_esp32\ardupilot-master`
- ArduRemoteID参考: `f:\opensource\usv_esp32\ArduRemoteID-master`

## 版本历史

### v1.0 (当前版本)
- 基础ESP32-S3支持
- ICM20948 IMU集成
- GPS支持 (u-blox MAX-M10S)
- 8MB PSRAM启用
- WiFi支持
- DroneCAN预留（待启用）

---

最后更新: 2025-10-29
