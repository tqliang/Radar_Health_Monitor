# Radar Health Monitor

基于 ESP32-S3 + Infineon BGT60TR13C 毫米波雷达的**非接触式生命体征检测系统**，可在 MCU 端实时计算呼吸率 (Breath Rate) 和心率 (Heart Rate)，并通过 UART 发送至上位机。

## 硬件需求

| 组件 | 型号 / 说明 |
|------|-------------|
| MCU | ESP32-S3 |
| 雷达传感器 | Infineon BGT60TR13C (60 GHz FMCW) |
| 连接方式 | SPI (10 MHz) + GPIO 中断 |

### GPIO 引脚映射

| 功能 | GPIO | 说明 |
|------|------|------|
| SPI CS | GPIO17 | 片选 (低电平有效) |
| SPI SCK | GPIO12 | 时钟 |
| SPI MOSI | GPIO14 | 主机输出 / 从机输入 |
| SPI MISO | GPIO15 | 主机输入 / 从机输出 |
| Radar IRQ | GPIO13 | 雷达 FIFO 水位线中断 (上升沿触发) |
| Radar Reset | GPIO16 | 雷达硬件复位 (低电平复位) |
| 3.3V→1.8V EN | GPIO1 | 雷达内部参考电压使能 |
| UART2 TX | GPIO39 | 呼吸/心率数据输出 (115200 bps) |
| UART2 RX | GPIO40 | 预留 |

## 雷达参数

| 参数 | 值 |
|------|-----|
| 工作频段 | 58 – 63 GHz |
| 采样点数 / Chirp | 128 |
| Chirp 数 / 帧 | 16 |
| RX 天线数 | 3 |
| 帧周期 | ~200 ms (约 5 Hz) |
| 每帧总采样点 | 128 × 16 × 3 = 6144 |
| FIFO 数据格式 | 12-bit 压缩 (3 字节 = 2 采样点) |
| SPI 速率 | 10 MHz |

## 工作原理

```
┌─────────────────────────────────────────────────────────┐
│  BGT60TR13C 雷达芯片                                     │
│  ┌─────────┐    FIFO 水位线到达 (CREF=528)               │
│  │ ADC → FIFO │────────────────────── IRQ (GPIO13) ────→ │
│  └─────────┘                                              │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│  ESP32-S3                                                │
│  ┌──────────────────────────────────────────────────┐   │
│  │ ISR: gpio_radar_isr_handler()                    │   │
│  │      → xSemaphoreGiveFromISR()                   │   │
│  └──────────────────┬───────────────────────────────┘   │
│                     ▼                                     │
│  ┌──────────────────────────────────────────────────┐   │
│  │ 雷达任务: xensiv_bgt60tr13c_radar_task()         │   │
│  │   1. 等待信号量 (xSemaphoreTake)                 │   │
│  │   2. FIFO 溢出/下溢检测                          │   │
│  │   3. SPI 读取 FIFO 原始数据                      │   │
│  │   4. 12-bit → 16-bit 数据解包                    │   │
│  │   5. 收满一帧 → vital_signs_process_frame()      │   │
│  │   6. 输出呼吸/心率 → UART2 发送                  │   │
│  └──────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

## 生命体征检测算法

- **呼吸频带**: 0.10 – 0.50 Hz (6 – 30 bpm)
- **心跳频带**: 0.80 – 2.50 Hz (48 – 150 bpm)
- **滑动窗口**: 呼吸 120 帧 (~24 s)、心跳 80 帧 (~16 s)
- **MTI 滤波**: 静态杂波抑制 (α = 0.98)
- **距离 bin 选择**: 自动选择最优 bin (5 – 80)

## UART2 输出协议

二进制定长帧，每帧 **16 字节**，约 1.4 ms @ 115200 bps：

```
字节:  0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15
     +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
     | 0xAA | 0x55 | 0x01 |    frame_cnt    |  breath_rate  |  heart_rate   |  CRC8  |
     +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
       HDR (固定)  FLAG    uint32 LE          float LE        float LE        MAXIM
```

| 字段 | 字节 | 类型 | 说明 |
|------|------|------|------|
| HDR | 0 – 1 | uint16 | 帧头 0xAA55 |
| FLAG | 2 | uint8 | 帧类型 0x01 |
| frame_cnt | 3 – 6 | uint32 LE | 帧序号 (自增) |
| breath_rate | 7 – 10 | float LE | 呼吸率 (bpm) |
| heart_rate | 11 – 14 | float LE | 心率 (bpm) |
| CRC8 | 15 | uint8 | MAXIM/DALLAS 多项式 0x31 |

## 工程结构

```
Radar/
├── CMakeLists.txt              # 顶层 CMake (ESP-IDF 项目)
├── sdkconfig                   # 项目配置 (已 .gitignore 排除)
├── README.md
├── include/
│   ├── bgt60tr13c_config.h     # 雷达寄存器配置 (Radar IDE 导出)
│   ├── bgt60tr13c_driver.h     # 雷达驱动函数声明
│   ├── bgt60tr13c_regs.h       # 寄存器地址 / 位掩码定义
│   ├── vital_signs.h           # 生命体征检测算法参数
│   └── uart2_radar.h           # UART2 输出协议定义
└── main/
    ├── CMakeLists.txt
    ├── main.c                  # 主程序 (SPI 初始化、中断、雷达任务)
    ├── bgt60tr13c_driver.c     # 雷达驱动 (SPI 读写、FIFO 读取、配置)
    ├── vital_signs.c           # 生命体征检测算法实现
    └── uart2_radar.c           # UART2 数据发送
```

## 快速开始

### 1. 环境准备

- [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/index.html)
- 目标芯片: ESP32-S3

### 2. 编译与烧录

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <端口> flash monitor
```

### 3. 查看输出

- 主串口 (UART0, 921600 bps): 调试日志 (帧序号、bin、呼吸/心率值)
- UART2 (GPIO39, 115200 bps): 二进制帧数据 (供上位机解析)

## 依赖

- ESP-IDF v5.x (SPI Master、GPIO、UART 驱动)
- FreeRTOS (任务调度、信号量)
- 无需外部组件库 (雷达驱动和算法均在工程内实现)