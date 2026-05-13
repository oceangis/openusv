# DroneCAN AHRS+GNSS 独立导航模块 设计文档

| 字段 | 值 |
|---|---|
| 项目名称 | USV-AHRS（暂定，待用户确认） |
| 节点 ID | `org.usvahrs.dronecan`（暂定） |
| 文档版本 | v0.1 |
| 创建日期 | 2026-04-25 |
| 目标定位 | 长期开源硬件产品 |
| 许可 | 固件 GPLv3，硬件 CERN-OHL-S |
| 上游目标 | `AP_ExternalAHRS_DroneCAN` backend 合并到 ArduPilot 主线 |

---

## 1. 项目目标

### 1.1 痛点

USV 主控（ESP32-S3）当前一体化运行 ArduPilot Rover：

- **EKF3 算力压力**：单核运行紧张，看门狗超时频发（已修复但脆弱）
- **磁罗盘干扰严重**：电机/电池/船架磁场污染导致航向跳变（参考 MEMORY.md `ICM-20948 AK09916 Compass Rotation Bug`）
- **校准疲劳**：每次出航前必须执行罗盘 8 字校准，每换水域/季节都要重做
- **代码耦合**：IMU 驱动、磁罗盘驱动、EKF 调参、振动滤波器深度耦合在主控代码中，维护成本高

### 1.2 目标

设计一个**独立 DroneCAN AHRS+GNSS 节点**，物理上远离主控的电磁干扰区，完整接管姿态估计和导航解，主控通过 DroneCAN 消费完整 NAV solution，**主控完全不再运行 EKF3 / IMU 采样 / 磁罗盘融合**。

### 1.3 非目标

- 不追求飞控级精度（VectorNav VN-300 那种 0.1°）
- 不做多 IMU 冗余
- 不做光流/视觉融合
- 不在模块上做控制律、任务管理、通信中转

### 1.4 作用域决策：扩展 AHRS（INS-like）

模块输出**完整导航解**，而非仅姿态+航向。具体输出维度：

| 维度 | 来源 | DroneCAN 消息 |
|---|---|---|
| Quaternion (姿态) | Mahony 融合 | `ahrs.Solution` |
| Angular velocity | gyro | `ahrs.Solution` |
| Linear acceleration | accel | `ahrs.Solution` |
| **Position (lat/lon/alt)** | AT310 GNSS | `gnss.Fix2` |
| **Velocity (NED)** | AT310 GNSS | `gnss.Fix2` |
| Heading (true yaw) | AT310 双天线 + Mahony | 包含在 quaternion |

**为什么选 INS-like 而非严格 AHRS**：

1. **一劳永逸**：硬件已包含 GNSS（AT310 双天线必须接 GNSS 才能工作），不发布 GNSS 位置等于浪费现成数据
2. **主控彻底解耦**：主控连 NMEA 解析都不用做，主控代码精简度最大化
3. **冗余路径清晰**：主控保留 u-blox MAX-M10S 作为备份 GPS，与模块 AT310 形成双源
4. **未来扩展性**：将来加视觉/雷达只需在模块端融合，主控接口不变
5. **符合 ExternalAHRS 标准模式**：VectorNav VN-300 / MicroStrain 系列都是这个作用域

**主控配置含义**：
- `EK3_ENABLE = 0`：完全关闭 EKF3
- `INS_ENABLE_MASK = 0`：不采主板 IMU
- `COMPASS_ENABLE = 0`：不读主板磁罗盘
- `GPS_TYPE = 1`（u-blox 自动）：保留主板 GPS 作为备份独立源
- `AHRS_EKF_TYPE = 11`：所有姿态+位置+速度从 ExternalAHRS 取

---

## 2. 系统架构

### 2.1 整体框图

```
┌─────────── AHRS 模块（桅杆顶部 / 船首） ──────────┐
│                                                    │
│   天线 A ─────► AT310-8A                           │
│                  (双天线定向)                      │
│                  ├── UART2 → STM32G474            │
│                  └── 1PPS → STM32G474 (TIM 输入)   │
│                                                    │
│   天线 B ─────►                                    │
│                                                    │
│   ICM-20948 ─── SPI1 ─► STM32G474                 │
│   (accel/gyro 高频)                                │
│                                                    │
│              STM32G474RET6                         │
│              ├── Mahony 滤波 (100Hz)               │
│              ├── AT310 yaw 修正                    │
│              ├── AP_Periph 框架                    │
│              └── FDCAN1 ─► DroneCAN                │
│                                                    │
└────────────────────┬───────────────────────────────┘
                     │ FDCAN @ 1Mbps
                     │ 标准 4 针 JST-GH 连接器
                     │
┌────────────────────▼───────────────────────────────┐
│              ESP32-S3 主控 (船舱内)                │
│                                                    │
│   AP_DroneCAN ─► AP_ExternalAHRS_DroneCAN (新)    │
│                  AHRS_EKF_TYPE = 11                │
│                                                    │
│   (无 EKF3, 无 ICM-20948 采样, 无 AK09916 融合)    │
│                                                    │
│   保留：                                            │
│   - u-blox MAX-M10S GPS（备份导航源）              │
│   - LoRa MAVLink 数传                              │
│   - WiFi 配置                                       │
│   - 任务规划、控制律、推进器输出                    │
│                                                    │
└────────────────────────────────────────────────────┘
```

### 2.2 数据流

| 频率 | 路径 | 内容 |
|---|---|---|
| 1125 Hz | ICM-20948 内部 ODR | 原始 accel/gyro |
| 200 Hz | ICM-20948 → G474 SPI | 中断驱动读取 |
| 100 Hz | G474 内部 | Mahony 迭代 |
| 1 Hz | AT310 → G474 UART | NMEA + PCAS50 |
| 1 Hz | G474 内部 | AT310 yaw 修正 Mahony 积分 |
| 50 Hz | G474 → CAN | `uavcan.equipment.ahrs.Solution` |
| 5 Hz | G474 → CAN | `uavcan.equipment.gnss.Fix2` |
| 1 Hz | G474 → CAN | `uavcan.protocol.NodeStatus` |
| 50 Hz | ESP32 主控 | 喂给 `AP_AHRS_External` |

### 2.3 故障降级矩阵

| 故障 | 模块行为 | 主控行为 |
|---|---|---|
| AT310 GNSS 失锁 | yaw 切到磁罗盘备份，pos 标记 DEAD_RECKON | 触发 RTL |
| AT310 双天线丢锁（仅单点） | yaw 切磁罗盘 + 警告 | 接受降级精度 |
| ICM-20948 SPI 失败 | 停发 AHRS，NodeStatus 报 ERROR | ExternalAHRS 超时 → failsafe HOLD |
| CAN 总线断 | 模块继续工作 | ExternalAHRS 超时 → failsafe RTL（用备份 GPS） |
| MCU 死机 | 看门狗复位（3s） | 检测节点离线 → failsafe |
| 主控备份 GPS 失锁 | 不影响 | AT310 仍提供位置（双源冗余） |

---

## 3. 硬件设计

### 3.1 主芯片：STM32G474RET6

| 参数 | 值 |
|---|---|
| 内核 | Cortex-M4F @ 170 MHz |
| Flash | 512 KB |
| SRAM | 128 KB |
| FPU | 单精度硬件 FPU |
| FDCAN | 3 路（用 FDCAN1） |
| UART | 5 路（用 USART1 调试 + USART2 接 AT310） |
| SPI | 4 路（用 SPI1 接 ICM-20948） |
| 定时器 | 17 个（其中 TIM2 捕获 AT310 1PPS） |
| 封装 | LQFP-64 |

**理由**：
- ArduPilot AP_Periph 官方支持
- 比 F303 富余（256K Flash 跑 AP_Periph 紧），后续加磁强计/气压计/温度补偿不慌
- 3× FDCAN 为未来扩展冗余 CAN 总线留余地
- ¥30 量产价

### 3.2 GNSS：AT310-8A（杭州中科微）

| 参数 | 值 |
|---|---|
| 系统 | BDS B1I/B1C + GPS L1C/A + Galileo E1 + QZSS L1 |
| 双天线定向精度 | 0.5°（基线 0.5m）/ 0.2°（基线 1m） |
| 单点定位精度 | 2.5m CEP50 |
| 速度精度 | 0.05 m/s |
| 数据更新率 | 1 Hz |
| 初始化时间 | 60s |
| 输出 | UART 115200, NMEA-0183 + CASBIN 二进制 |
| 1PPS 输出 | 20ns 抖动（用于精确时间同步） |
| 供电 | 3.3V，<200mW |
| 尺寸 | 22×24×3 mm |

**关键配置（出厂烧录）**：
- `PCAS08`：基线长度 = 实测值（精度 ±2cm）
- `PCAS09`：yaw 输出范围 [-180, 180]
- `cpitchMode = 2`：使用 IMU 倾斜约束（提高动态精度）

### 3.3 IMU：ICM-20948（TDK InvenSense）

| 参数 | 值 |
|---|---|
| 类型 | 9 轴（3 轴 accel + 3 轴 gyro + 3 轴 mag AK09916） |
| Accel 范围 | ±2/4/8/16 g（用 ±8g） |
| Accel 噪声 | 230 μg/√Hz |
| Gyro 范围 | ±250/500/1000/2000 dps（用 ±2000 dps） |
| Gyro 噪声 | 0.015 dps/√Hz |
| 接口 | **SPI**（不用 I2C，桅杆走线长 SPI 抗噪好） |
| 时钟 | 7 MHz（< MCLK/10） |

**安装方位**：模块外壳 X 轴对齐船头方向，Z 轴朝天，遵循 NED 默认约定（无需 ROTATION 修正）。

**磁罗盘 AK09916**：保留可读取，作为 AT310 失锁时的备份 yaw 源。**正常工况不参与融合**，避开磁干扰复杂性。

### 3.4 PCB 布局

**尺寸**：50 × 50 × 12 mm（含外壳）

**分区**：
```
┌─────────────────────────────────────┐
│  ANT_A (SMA)         ANT_B (SMA)    │ ← 通过外接电缆引到外壳两侧
│   │                    │            │
├───┼────────────────────┼────────────┤
│   │   AT310-8A (RF 区) │            │
│   └────────────────────┘            │
│      ┌────────────┐                 │
│      │ STM32G474  │  ← 数字区        │
│      │            │                 │
│      └────────────┘                 │
│      ┌──────────┐                   │
│      │ICM-20948 │  ← 远离 G474 主电源│
│      └──────────┘    布在 PCB 中心    │
│                                      │
│   电源区 (DC-DC + LDO)                │
│   CAN 收发器 (TJA1051 / SN65HVD230)   │
│                                      │
│   JST-GH 4P (DroneCAN)                │
└──────────────────────────────────────┘
```

**关键布局规则**：
- AT310 RF 区下方完整地平面，禁止走信号
- ICM-20948 远离 DC-DC 开关电源 ≥ 10mm
- ICM-20948 下方放置完整地平面，加一圈"井字"过孔屏蔽
- 双天线 SMA 走线 50Ω 阻抗匹配（PCB 介电常数计算线宽）

### 3.5 接口定义

#### 3.5.1 DroneCAN 连接器（JST-GH 4 针）

兼容 Pixhawk/CubePilot 标准：

| Pin | 信号 | 备注 |
|---|---|---|
| 1 | VBAT (5V-36V) | 宽压输入，模块内部 DC-DC |
| 2 | CAN_H | 差分高 |
| 3 | CAN_L | 差分低 |
| 4 | GND | |

**终端电阻**：120Ω，板载焊跨接（默认开），用户根据总线位置决定。

#### 3.5.2 GNSS 天线接口

- 2 × SMA 母座（外壳侧）
- 通过 RG178 同轴电缆接到 PCB IPEX 座
- 板上有源天线供电 3.3V，串保险电阻

#### 3.5.3 调试接口（量产版可省）

- SWD 4 针（电源 + GND + SWDIO + SWCLK）
- USART1 TTL 3 针（GND + TX + RX）：调试日志

### 3.6 电源架构

```
VBAT (5V-36V CAN bus)
    │
    ▼
TPS62A0518 DC-DC ──► 3.3V_DIG (200mA, 给 G474 + ICM-20948)
    │
    └─► LDO ──► 3.3V_RF (100mA, 单独给 AT310 + 天线)
```

**关键点**：
- AT310 RF 电源**单独 LDO**，避免 DC-DC 纹波串扰 GNSS（datasheet 要求 < 50mVpp）
- 数字电源单星地，磁珠隔离 ICM-20948 电源
- VBAT 输入 TVS 二极管（SMAJ36CA）防浪涌

### 3.7 BOM（核心元件）

| 元件 | 型号 | 单价 | 备注 |
|---|---|---|---|
| MCU | STM32G474RET6 | ¥30 | LQFP64 |
| GNSS | AT310-8A-36 | ¥120 | 含双天线定向 |
| IMU | ICM-20948 | ¥25 | LGA-24 |
| CAN PHY | TJA1051T/3 | ¥3 | SOIC-8 |
| DC-DC | TPS62A0518 | ¥5 | SOT-23-6 |
| LDO | TLV75533 | ¥2 | 3.3V/500mA |
| SMA 座 | 通用 | ¥3×2 | |
| GNSS 天线 | 蘑菇头有源 L1 | ¥30×2 | 用户自配 |
| 外壳 | 铝合金 IP67 | ¥40 | 量产开模 |
| **整机** | | **~¥330** | |

---

## 4. 固件设计

### 4.1 框架选择：AP_Periph

固件基于 ArduPilot AP_Periph 框架开发，复用：
- ChibiOS RTOS + AP_HAL_ChibiOS
- DroneCAN 协议栈（libcanard）
- 参数系统（持久化到 G474 内部 Flash）
- 标准 Bootloader（支持 MissionPlanner OTA）
- NodeStatus / NodeInfo / GetSet / RestartNode 标准服务

### 4.2 hwdef.dat 关键配置

文件路径：`ArduPilot/Tools/AP_Periph/hwdef/usvahrs-g474/hwdef.dat`

```
# MCU class and specific type
MCU STM32G4xx STM32G474xx

# crystal frequency
OSCILLATOR_HZ 24000000

# board ID for firmware load (TBD: 需向 ArduPilot 申请正式 ID
# 参考 https://github.com/ArduPilot/ardupilot/blob/master/Tools/AP_Bootloader/board_types.txt
# 1300 仅为占位)
APJ_BOARD_ID 1300

FLASH_SIZE_KB 512
FLASH_RESERVE_START_KB 36

# bootloader is 36k
FLASH_BOOTLOADER_LOAD_KB 36

define HAL_STORAGE_SIZE 16384

# CAN 配置
PA11 CAN1_RX CAN1
PA12 CAN1_TX CAN1
PB10 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

# UART for AT310
PA2  USART2_TX USART2 SPEED_HIGH NODMA
PA3  USART2_RX USART2 SPEED_HIGH NODMA

# UART for debug
PA9  USART1_TX USART1 SPEED_HIGH NODMA
PA10 USART1_RX USART1 SPEED_HIGH NODMA

# SPI for ICM-20948
PA5  SPI1_SCK SPI1
PA6  SPI1_MISO SPI1
PA7  SPI1_MOSI SPI1
PB0  CS_ICM_20948 CS

# IMU interrupt
PB1  IMU_INT INPUT FLOATING

# 1PPS from AT310
PA0  TIM2_CH1 TIM2 INPUT GPIO(50)

# Status LED
PC13 LED OUTPUT LOW GPIO(0)

# AT310 NMEA serial
SERIAL_ORDER USART1 USART2

# IMU configuration (ICM-20948 属 Invensense v2 族)
SPIDEV icm20948 SPI1 DEVID1 CS_ICM_20948 MODE3 1*MHZ 7*MHZ
IMU Invensensev2 SPI:icm20948 ROTATION_NONE

# Compass (optional, AT310 yaw is primary)
COMPASS AK09916:probe_ICM20948 0 ROTATION_PITCH_180_YAW_90

# AHRS Solution publication
define HAL_PERIPH_ENABLE_AHRS
define HAL_PERIPH_ENABLE_GPS
define HAL_PERIPH_ENABLE_MAG

# Disable unused features
define HAL_PERIPH_NO_BARO 1
define HAL_PERIPH_NO_AIRSPEED 1
```

### 4.3 主任务循环

```c
// AP_Periph_FW.cpp 中的 update() 增加：

void AP_Periph_FW::update_ahrs() {
    // 200Hz: read IMU
    if (imu_ready()) {
        Vector3f accel, gyro;
        read_icm20948(&accel, &gyro);

        // 100Hz: Mahony predict
        mahony_update_imu(&filter, gyro, accel, dt);
    }

    // 1Hz: process AT310
    if (at310_has_new_solution()) {
        at310_solution_t sol;
        at310_get_solution(&sol);

        if (sol.fix_status == 3) {  // 双天线固定解
            // yaw correction
            mahony_correct_yaw(&filter, sol.yaw_deg, dt);
        }

        // GNSS Fix2 直接发布
        publish_gnss_fix2(&sol);
    }

    // 50Hz: publish AHRS solution
    if (now - last_ahrs_pub_ms >= 20) {
        publish_ahrs_solution(&filter);
        last_ahrs_pub_ms = now;
    }
}
```

### 4.4 Mahony 滤波器

伪代码（实际 ~80 行 C）：

```c
typedef struct {
    Quaternion q;       // 当前姿态四元数
    Vector3f bias;      // gyro 偏置估计
    float Kp, Ki;       // PI 增益
} mahony_t;

void mahony_update_imu(mahony_t *m,
                       Vector3f gyro,
                       Vector3f accel,
                       float dt) {
    // 1. 归一化 accel
    accel.normalize();

    // 2. 当前姿态预测的重力方向
    Vector3f v = m->q.to_gravity_vec();

    // 3. 误差 = 测量重力 × 预测重力
    Vector3f e = accel.cross(v);

    // 4. PI 反馈
    m->bias += e * m->Ki * dt;
    Vector3f gyro_corr = gyro + e * m->Kp + m->bias;

    // 5. 四元数积分
    m->q.rotate(gyro_corr * dt);
    m->q.normalize();
}

void mahony_correct_yaw(mahony_t *m, float at310_yaw_rad, float dt) {
    float current_yaw = m->q.yaw();
    float yaw_error = wrap_PI(at310_yaw_rad - current_yaw);

    // 1Hz 修正：低增益避免抖动
    float K_yaw = 0.5f;
    Vector3f yaw_correction(0, 0, yaw_error * K_yaw);

    m->q.rotate(yaw_correction * dt);
    m->q.normalize();
}
```

### 4.5 AT310 NMEA 解析

需要解析的句子：
- `$GPGGA` / `$GNGGA`：位置 + 海拔
- `$GPRMC` / `$GNRMC`：时间 + 速度 + 航向（仅参考）
- `$GPVTG` / `$GNVTG`：地速 + 真航向
- `$PCAS50`：双天线 yaw + pitch + roll + 定位质量 FS

`PCAS50` 是核心：
```
$PCAS50,UTCtime,Yaw,Pitch,Roll,Bl,FS,Nsv,PDOP,DegStd*CS<CR><LF>
```

**FS 字段**（定位质量）：
- 0 = 无效解 → 不可用
- 1 = 单点解 → yaw 不可信
- 2 = 浮点解 → yaw 精度退化（>1°）
- **3 = 固定解 → yaw 高精度，可用于融合**
- 4 = 外部输入解
- 6 = 估算解

**只有 FS == 3 时 yaw 才进入融合循环**，否则跳过修正。

### 4.6 DroneCAN 消息映射

| 主题 | 消息类型 | 频率 | 内容 |
|---|---|---|---|
| `uavcan.equipment.ahrs.Solution` | 1.7 | 50 Hz | quaternion + angular velocity + linear acceleration |
| `uavcan.equipment.gnss.Fix2` | 1.0 | 5 Hz | lat/lon/alt + velocity NED + status |
| `uavcan.equipment.gnss.Auxiliary` | 1.0 | 1 Hz | HDOP/VDOP + sats visible |
| `uavcan.equipment.ahrs.MagneticFieldStrength2` | 1.0 | 10 Hz | mag field（仅备份用，可关） |
| `uavcan.protocol.NodeStatus` | 1.0 | 1 Hz | 健康状态 |
| `uavcan.protocol.GetNodeInfo` | 1.0 | 响应 | 固件版本、硬件版本 |

### 4.7 启动序列

```
T+0s:   上电，bootloader 启动
T+0.5s: 跳转到应用，AP_Periph 框架初始化
T+1s:   ICM-20948 SPI 探测、配置
T+2s:   AT310 UART 打开，等待 NMEA
T+5s:   gyro 静态零偏估计（需 5s 静止）
T+5s:   开始发布 AHRS Solution（标志 INITIALIZING）
T+60s:  AT310 双天线收敛（FS=3），开始 yaw 修正
T+60s:  NodeStatus = OPERATIONAL
```

主控显示"AHRS 就绪"才允许 ARM。

---

## 5. 主控集成（ESP32-S3 / ArduPilot）

### 5.1 新建 backend：AP_ExternalAHRS_DroneCAN

代码位置：`libraries/AP_ExternalAHRS/AP_ExternalAHRS_DroneCAN.cpp/.h`

订阅：
- `uavcan.equipment.ahrs.Solution`
- `uavcan.equipment.gnss.Fix2`
- `uavcan.equipment.gnss.Auxiliary`

实现接口：
- `update()`：定期检查消息超时
- `get_filter_status()`：返回融合状态
- `get_quaternion()` / `get_velocity_NED()` / `get_origin()` / `get_location()`

数据流：
```
DroneCAN 消息 → AP_DroneCAN 回调
            → AP_ExternalAHRS_DroneCAN 缓存
            → AP_ExternalAHRS::update_state()
            → AP_AHRS 通过 ExternalAHRS 接口读取
```

### 5.2 主控参数变更

```
# 启用 ExternalAHRS
AHRS_EKF_TYPE = 11

# 关闭所有 EKF
EK2_ENABLE = 0
EK3_ENABLE = 0

# 关闭主板 IMU 采样
INS_ENABLE_MASK = 0

# 关闭主板磁罗盘
COMPASS_ENABLE = 0

# ExternalAHRS 配置
EAHRS_TYPE = 5             # DroneCAN backend (新增, TBD: 上游申请正式编号；
                           # 当前 1=VectorNav, 2=LordMicroStrain, 3=BNO08x, 4=InertialLabs, 5=未占用)
EAHRS_RATE = 50            # 50Hz 期望
EAHRS_OPTIONS = 0          # bitmask 保留

# DroneCAN 启用
CAN_P1_DRIVER = 1
CAN_D1_PROTOCOL = 1        # DroneCAN
CAN_D1_UC_NODE = 10        # 主控节点 ID
```

模块节点 ID 通常为 ≥ 100，避开主控范围。

### 5.3 主控代码精简清单

可以**禁用编译**或**完全删除**的文件：
- `libraries/AP_NavEKF3/*`（保留代码但 EK3_ENABLE=0 即可，节省编译时间可彻底排除）
- 主板 hwdef 中 ICM-20948 探测条目
- 主板 hwdef 中 AK09916 magnetometer 配置
- ICM-20948 振动滤波 / Notch filter 调参逻辑

### 5.4 备份方案：主控 GPS 保留

主控保留 u-blox MAX-M10S（UART1）作为：
- AHRS 模块全失效时的应急导航源
- 与 AT310 位置交叉验证（两个独立 GPS 源）
- failsafe RTL 时的最后一道保险

EKF3 关闭后，主控 GPS 直接喂给 ExternalAHRS 的"备份"通道（需小改 ExternalAHRS 抽象）。

---

## 6. 测试与验证

### 6.1 单元测试（板载）

| 测试项 | 方法 | 通过标准 |
|---|---|---|
| ICM-20948 通信 | SPI 读 WHO_AM_I (0x68) | 返回 0xEA |
| AT310 通信 | UART 收 NMEA 句 | 1 秒内收到 GGA |
| 1PPS 检测 | TIM2 输入捕获 | 1Hz ±10ppm |
| FDCAN 自环 | 内部 loopback | 收发匹配 |
| Mahony 收敛 | 静置 30 秒 | pitch/roll < 0.5° |

### 6.2 集成测试（台架）

| 场景 | 方法 | 通过标准 |
|---|---|---|
| 静态航向 | 转盘旋转已知角度 | 误差 < 1° |
| 动态摇摆 | 手摇模块 ±30° | Mahony 跟踪不漂 |
| 振动 | 50-200Hz 正弦扫频 | 姿态噪声 < 1° RMS |
| AT310 yaw 修正 | 故意让 Mahony 偏 5°，等 1s | 修正后误差 < 0.5° |
| GNSS dropout | 拔天线 60s | 推算误差 < 0.2° |
| CAN 流量 | 50Hz AHRS + 5Hz GNSS | 总线占用 < 30% |

### 6.3 主控集成测试

| 场景 | 方法 | 通过标准 |
|---|---|---|
| ExternalAHRS 解锁 | ArduPilot ARM 流程 | 模块就绪后 ARM 通过 |
| 控制律输入 | MissionPlanner 看 attitude | quaternion 实时更新 |
| GNSS Fix | MissionPlanner 看 position | 与备份 GPS 一致（< 5m） |
| 模块掉线 | 拔 CAN 线 | 主控 5s 内进入 failsafe |
| 模块重连 | 重接 CAN 线 | 模块自动注册并恢复 |

### 6.4 USV 实船测试

| 测试 | 通过标准 |
|---|---|
| 静态停泊 1 小时 | yaw 漂 < 0.5° |
| 直线航行 100m × 5 次 | 轨迹偏差 < 1m |
| 圆周航行 r=10m | 闭合误差 < 2m |
| 满速转向 | 无姿态跳变 |
| 桥下短暂失锁（30s） | 推算后恢复正常 |
| 跨水域（淡水→咸水） | 无需校准，直接工作 |

---

## 7. 开源策略

### 7.1 仓库结构

```
usvahrs-dronecan/
├── hardware/
│   ├── kicad/                  # KiCad 工程
│   ├── gerber/                 # 量产 Gerber
│   ├── bom.csv                 # 元件清单
│   └── enclosure.step          # 外壳 STEP
├── firmware/
│   ├── hwdef.dat               # ArduPilot AP_Periph 板型定义
│   ├── ap_periph_patch/        # 必要的 ArduPilot patch
│   └── README.md
├── upstream-patches/
│   ├── ap_externalahrs_dronecan.patch
│   └── PR_TEMPLATE.md
├── docs/
│   ├── user_manual.md
│   ├── integration_guide.md
│   └── parameter_reference.md
└── LICENSE
```

### 7.2 上游策略

1. **Phase 1**: 项目自维护 fork，验证可用
2. **Phase 2**: 准备 ArduPilot PR：
   - `AP_ExternalAHRS_DroneCAN` backend
   - hwdef for `usvahrs-g474`
   - 文档更新
3. **Phase 3**: 跟随 ArduPilot 主线发布

### 7.3 许可

| 内容 | 许可 |
|---|---|
| 固件代码 | GPLv3（强制，因为引用 ArduPilot） |
| PCB 设计 | CERN-OHL-S v2 |
| 文档 | CC-BY-SA 4.0 |
| AT310 datasheet | 厂商版权（不重发布） |

---

## 8. 路线图

| 里程碑 | 工作内容 | 预计时间 |
|---|---|---|
| **M1: 原理图 + 选型** | 完成原理图、BOM 确认 | 1 周 |
| **M2: PCB Layout** | 双层/四层 PCB 布板，发样 | 1 周 |
| **M3: 打板回来** | 焊接、上电、烟测 | 2 周 |
| **M4: 固件 Bring-up** | hwdef + 驱动跑通 | 1 周 |
| **M5: Mahony 集成** | 滤波器调通，AT310 接入 | 1 周 |
| **M6: DroneCAN 集成** | 标准消息发布、MissionPlanner 识别 | 1 周 |
| **M7: 主控 Backend** | `AP_ExternalAHRS_DroneCAN` 写完 | 1 周 |
| **M8: 联调测试** | 台架完整测试 | 1 周 |
| **M9: USV 实船** | 实际水域测试 | 2 周 |
| **M10: 开源发布** | GitHub 发布 + 上游 PR | 1 周 |
| **总计** | | **~12 周** |

---

## 9. 风险登记

| 风险 | 概率 | 影响 | 应对 |
|---|---|---|---|
| AT310 实际更新率 1Hz 真的够吗 | 低 | 中 | 已论证；保留升级到 5Hz 的厂家询问余地 |
| ICM-20948 SPI 走线长被电磁干扰 | 中 | 中 | PCB 走 50Ω 阻抗 + 屏蔽地、限制线长 |
| AP_ExternalAHRS_DroneCAN backend 上游不接 | 中 | 低 | 即便不接，自维护 patch 不大 |
| AT310 60s 启动延迟用户抱怨 | 高 | 低 | 文档提示 + 主控显示就绪指示 |
| 双天线基线安装不便（小船） | 中 | 中 | 提供刚性桅杆 STL，鼓励用户自打印 |
| 多径干扰（码头、护栏） | 中 | 低 | FS<3 时跳过 yaw 修正，降级到 gyro 推算 |

---

## 10. 待用户确认事项

### 已决策
- ✅ MCU：STM32G474RET6
- ✅ GNSS：AT310-8A
- ✅ IMU：ICM-20948
- ✅ 融合算法：Mahony filter（不上 EKF3）
- ✅ 框架：ArduPilot AP_Periph
- ✅ DroneCAN 路径：路径 1（写新 backend，目标合上游）
- ✅ 作用域：扩展 AHRS（INS-like，含 GNSS 位置+速度）
- ✅ 主控备份 GPS：保留 u-blox MAX-M10S
- ✅ 项目定位：长期开源硬件产品
- ✅ 基线长度：能拉开 ≥0.5m

### 待确认
1. **节点名**：`org.usvahrs.dronecan` 接受？或自定义？（影响烧录）
2. **外壳/防水等级**：IP67 铝壳 vs IP54 塑料壳 vs 工业级？
3. **目标量产价位**：< ¥300 / < ¥500 / < ¥800？影响选型余地
4. **预算时间**：12 周路线图能接受吗？需要并行加速吗？
5. **PCB 设计软件**：KiCad（推荐开源）/ AD / EAGLE？
6. **ArduPilot APJ_BOARD_ID**：需向上游申请正式 ID
7. **EAHRS_TYPE 枚举值**：需向上游申请编号
