# Wheel-Legged Robot Control System | 轮足机器人控制系统

[![Platform](https://img.shields.io/badge/Platform-STM32F407-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407-417.html)
[![Framework](https://img.shields.io/badge/Framework-STM32CubeMX_6.15.0-green.svg)](https://www.st.com/en/development-tools/stm32cubemx.html)
[![Language](https://img.shields.io/badge/Language-C-orange.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**English** | [中文](#中文文档)

---

## Overview

This project implements the embedded control system for a wheel-legged robot, developed for the **2026 Mingyue Class**. The system is based on the STM32F407IGHx microcontroller and supports multiple motor types through CAN and UART communication interfaces.

### Key Features

- **Multi-Motor Support**: EL05 (CAN extended frame), M1502E (CAN standard frame), M0601C (UART)
- **Multiple Control Modes**: MIT mode, Position, Velocity, Current control
- **Real-time Communication**: CAN 2.0 @ 500Kbps, UART @ 115200bps with DMA
- **STM32 HAL Framework**: Built with STM32CubeMX generated code
- **Modular Architecture**: Independent driver modules for easy integration

---

## Hardware Requirements

### Main Controller

| Component | Specification |
|-----------|---------------|
| MCU | STM32F407IGHx (UFBGA176 package) |
| Core | ARM Cortex-M4 @ 168 MHz with FPU |
| CAN Interface | CAN1 (PD0: RX, PD1: TX) |
| UART Interface | USART1 (PB7: RX, PA9: TX) with DMA |
| Debug Interface | SWD (PA13: SWDIO, PA14: SWCLK) |
| External Oscillator | 12 MHz HSE |

### Supported Motors and Modules

#### NRF24L01+ Wireless Module (Remote Control)

| Parameter | Value |
|-----------|-------|
| Operating Voltage | 1.9V - 3.6V (3.3V recommended) |
| Frequency Range | 2.400GHz - 2.525GHz |
| Data Rate | 250kbps (configured) |
| TX Power | -6dBm (configured) |
| Communication | SPI interface (SPI3) |
| Range | Up to 100m (open area) |
| Features | Auto-acknowledgment, auto-retransmit, pairing protocol |

**Hardware Connection:**
```
NRF24L01 Module      STM32F407
─────────────────────────────
VCC         ───►   3.3V (⚠️ NOT 5V!)
GND         ───►   GND
CE          ───►   PC8
CSN         ───►   PC9
SCK         ───►   PC10 (SPI3_SCK)
MOSI        ───►   PC12 (SPI3_MOSI)
MISO        ───►   PC11 (SPI3_MISO)
IRQ         ───►   PC7 (optional)
```

**Pairing Protocol:**
- Default pairing address: `"HXFB0"`
- New address after pairing: `"HXFB1"`
- RF Channel: 25 (2.425GHz)
- Payload width: 16 bytes
- Auto-retransmit: 8 retries, 2000µs delay

#### EL05 Quasi-Direct-Drive Motor (CAN Extended Frame)

| Parameter | Value |
|-----------|-------|
| Rated Voltage | 48V DC (15V-60V range) |
| Rated Torque | 1.8 N·m |
| Peak Torque | 6 N·m (5s duration) |
| No-load Speed | 430 rpm |
| Gear Ratio | 9:1 |
| Communication | CAN 2.0 Extended Frame @ 500Kbps |
| Control Modes | MIT, Position (PP/CSP), Velocity, Current |

#### M1502E Motor (CAN Standard Frame)

| Parameter | Value |
|-----------|-------|
| Communication | CAN 2.0 Standard Frame @ 500Kbps |
| Control Modes | Velocity mode |
| ID Range | 1-4 (configurable via CAN) |

#### M0601C Motor (UART)

| Parameter | Value |
|-----------|-------|
| Communication | UART @ 115200 bps, 8N1 |
| Frame Length | 10 bytes (with CRC-8/MAXIM) |
| Control Modes | Current, Speed, Position |
| ID Range | 1-4 |

---

## Project Structure

```
Wheel-Legged_Robot/
├── Core/                              # Main application code
│   ├── Inc/                           # Header files
│   │   ├── main.h
│   │   ├── can.h
│   │   └── gpio.h
│   └── Src/                           # Source files
│       ├── main.c                     # Main entry point
│       ├── can.c                      # CAN init & M1502E driver
│       └── gpio.c                     # GPIO configuration
│
├── Motor_Drivers/                     # Motor driver modules
│   ├── EL05_MOTOR_DRIVE/              # EL05 CAN motor driver
│   │   ├── Core/
│   │   │   ├── Inc/
│   │   │   │   ├── el05_motor.h       # EL05 driver API
│   │   │   │   └── can_rx_handler.h   # CAN RX handler
│   │   │   └── Src/
│   │   │       ├── el05_motor.c       # EL05 implementation
│   │   │       ├── can_rx_handler.c   # Feedback parsing
│   │   │       └── main_example.c     # Usage examples
│   │   └── EL05电机驱动使用指南.md     # Chinese documentation
│   │
│   ├── M0601C_DRIVE/                  # M0601C UART motor driver
│   │   ├── Core/
│   │   │   ├── Inc/
│   │   │   │   └── motor_driver.h     # M0601C driver API
│   │   │   └── Src/
│   │   │       └── motor_driver.c     # M0601C implementation
│   │   └── *.md                       # Documentation files
│   │
│   └── NRF24L01_DRIVER/               # NRF24L01+ wireless module driver
│       ├── Core/
│       │   ├── Inc/
│       │   │   └── nrf24l01.h         # NRF24L01 driver API
│       │   └── Src/
│       │       ├── nrf24l01.c         # NRF24L01 implementation
│       │       └── nrf24l01_example.c # Usage examples
│       └── README.md                  # Documentation
│
├── Drivers/                           # STM32 HAL & CMSIS libraries
├── MDK-ARM/                           # Keil MDK project files
├── EWARM/                             # IAR EWARM project files
└── WheelRobot.ioc                     # STM32CubeMX configuration
```

---

## Quick Start

### 1. Hardware Connection

**EL05 Motor (CAN Extended Frame):**
```
EL05 Motor          STM32F407
─────────────────────────────
CAN_H        ───►   CAN_H (PD1)
CAN_L        ───►   CAN_L (PD0)
GND          ───►   GND
48V DC       ───►   Power Supply (15V-60V)

Note: Add 120Ω termination resistor at both ends of CAN bus
```

**M1502E Motor (CAN Standard Frame):**
```
M1502E Motor        STM32F407
─────────────────────────────
CAN_H        ───►   CAN_H (PD1)
CAN_L        ───►   CAN_L (PD0)
GND          ───►   GND

Note: Shares the same CAN bus with EL05
```

**M0601C Motor (UART):**
```
M0601C Motor        STM32F407
─────────────────────────────
TX           ───►   USART1_RX (PB7)
RX           ───►   USART1_TX (PA9)
GND          ───►   GND
```

### 2. Software Build

#### Using Keil MDK-ARM

1. Open `MDK-ARM/WheelRobot.uvprojx`
2. Select target configuration
3. Build project (F7)
4. Flash to target (F8)

#### Using IAR EWARM

1. Open `EWARM/WheelRobot.eww`
2. Build and download

### 3. Code Integration

**EL05 Motor Example:**

```c
#include "el05_motor.h"

EL05_MotorHandle_t motor1;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();

    // Initialize EL05 driver
    EL05_Init(&hcan1);
    EL05_StartReception();

    // Configure motor
    motor1.can_id = 0x01;
    EL05_Enable(&motor1);
    HAL_Delay(100);

    // MIT mode control
    EL05_MitControl_t cmd = {
        .p_des = 0.0f,   // Target position (rad)
        .v_des = 1.0f,   // Target velocity (rad/s)
        .kp = 10.0f,     // Position gain
        .kd = 0.5f,      // Velocity gain
        .t_ff = 0.0f     // Feedforward torque (N·m)
    };

    while (1) {
        EL05_MitControl(&motor1, &cmd);
        HAL_Delay(10);  // 100Hz control rate
    }
}
```

**M1502E Motor Example:**

```c
#include "can.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();

    // Switch to velocity mode
    M1502E_SetSpeedMode();
    HAL_Delay(100);

    // Set motor velocity (RPM)
    M1502E_SetVelocity(30);  // 30 RPM

    while (1) {
        // Main loop
    }
}
```

**M0601C Motor Example:**

```c
#include "motor_driver.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    // Start receiving motor feedback
    MOTOR_StartReceive();

    // Set motor speed (Motor ID 1, 30 RPM)
    MOTOR_SetSpeed(1, 30);

    while (1) {
        // Get motor status
        MotorStatus_t *status = MOTOR_GetStatus();
        // Use status->speed, status->current, etc.
        HAL_Delay(100);
    }
}
```

---

## API Reference

### EL05 Motor Driver (CAN Extended Frame)

| Function | Description |
|----------|-------------|
| `EL05_Init(hcan)` | Initialize driver with CAN handle |
| `EL05_StartReception()` | Start CAN RX interrupts |
| `EL05_Enable(motor)` | Enable motor |
| `EL05_Disable(motor)` | Disable motor |
| `EL05_SetMode(motor, mode)` | Set control mode |
| `EL05_MitControl(motor, cmd)` | MIT mode control |
| `EL05_PositionControl(motor, pos, vel_limit)` | Position control |
| `EL05_VelocityControl(motor, vel, cur_limit)` | Velocity control |
| `EL05_CurrentControl(motor, cur)` | Current control |
| `EL05_SetZeroPosition(motor)` | Set mechanical zero |
| `EL05_WriteParam(motor, addr, value)` | Write parameter |
| `EL05_ReadParam(motor, addr)` | Read parameter |
| `EL05_GetFeedback(motor)` | Get feedback data |
| `EL05_CheckOnline(motor, timeout)` | Check motor online status |

### M1502E Motor Driver (CAN Standard Frame)

| Function | Description |
|----------|-------------|
| `M1502E_SetSpeedMode()` | Switch motor to velocity mode |
| `M1502E_SetVelocity(rpm)` | Set motor velocity (RPM) |
| `M1502E_Config_ID(id)` | Set motor ID (1-8) |

### M0601C Motor Driver (UART)

| Function | Description |
|----------|-------------|
| `MOTOR_CRC8_Calc(data, len)` | Calculate CRC-8/MAXIM |
| `MOTOR_SendDriveCmd(id, value, acc, brake)` | Send drive command |
| `MOTOR_SetSpeed(id, rpm)` | Set motor speed |
| `MOTOR_Brake(id)` | Motor brake |
| `MOTOR_Stop(id)` | Motor stop |
| `MOTOR_SendModeSwitchCmd(id, mode)` | Switch control mode |
| `MOTOR_SendSetIDCmd(newId)` | Set motor ID |
| `MOTOR_StartReceive()` | Start UART DMA RX |
| `MOTOR_GetFeedback()` | Get raw feedback |
| `MOTOR_GetStatus()` | Get parsed status |

### NRF24L01+ Wireless Module Driver

| Function | Description |
|----------|-------------|
| `NRF24L01_RX_Init()` | Initialize receiver with default address |
| `NRF24L01_RX_WaitForPairing()` | Wait for remote pairing (10s timeout) |
| `NRF24L01_RX_ReadData()` | Read data from remote (auto-response) |
| `NRF24L01_RX_GetData()` | Get parsed remote control data |
| `NRF24L01_RX_IsOnline()` | Check remote online status |
| `NRF24L01_RX_IsPaired()` | Check pairing status |

**Remote Control Data Structure:**
```c
typedef struct {
    uint8_t right_joystick_x;   // 128=center
    uint8_t right_joystick_y;   // 128=center
    uint8_t left_joystick_x;    // 128=center
    uint8_t left_joystick_y;    // 128=center
    uint8_t button_state;       // Button bitmask
    uint8_t rolling_code;       // Rolling code
    uint8_t data_valid;         // Data valid flag
    uint32_t timestamp;         // Reception timestamp (ms)
} RemoteControlData_t;
```

**Usage Example:**
```c
#include "nrf24l01_rx.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI3_Init();

    // Initialize NRF24L01 receiver
    NRF24L01_RX_Init();

    // Wait for pairing (10s timeout)
    if (!NRF24L01_RX_WaitForPairing()) {
        // Pairing failed
        while(1);
    }

    while (1) {
        // Read remote data
        if (NRF24L01_RX_ReadData()) {
            RemoteControlData_t *rc = NRF24L01_RX_GetData();

            // Use joystick data (128 = center)
            int16_t right_x = rc->right_joystick_x - 128;
            int16_t right_y = rc->right_joystick_y - 128;
            int16_t left_x = rc->left_joystick_x - 128;
            int16_t left_y = rc->left_joystick_y - 128;

            // Check buttons
            if (rc->button_state & 0x01) {
                // KEY1 pressed
            }
        }

        // Check online status
        if (!NRF24L01_RX_IsOnline()) {
            // Remote offline - stop motors
        }

        HAL_Delay(10);  // 100Hz update rate
    }
}
```

---

## Control Modes

### EL05 MIT Mode (运控模式)

The most flexible control mode for robotics applications:

```
Output Torque = Kp × (p_des - p_actual) + Kd × (v_des - v_actual) + t_ff
```

**Parameter Ranges:**

| Parameter | Range | Unit |
|-----------|-------|------|
| p_des | -12.5 ~ 12.5 | rad |
| v_des | -30 ~ 30 | rad/s |
| kp | 0 ~ 500 | - |
| kd | 0 ~ 5 | - |
| t_ff | -18 ~ 18 | N·m |

---

## Safety Notes

### EL05 Motor

- **Operating Voltage**: 15V - 60V DC (Rated: 48V)
- **Maximum Torque Duration**:

| Torque (N·m) | Duration (s) |
|--------------|--------------|
| 6 | 5 |
| 5 | 7 |
| 4 | 14 |
| 3 | 44 |
| 2 | 300 |
| 1.8 | Continuous |

- **Temperature Protection**: Auto shutdown at 145°C

### General Safety

- Always check motor connections before powering on
- Ensure proper CAN termination (120Ω at both ends)
- Monitor motor temperature during operation
- Start with low gains and increase gradually
- Never exceed rated voltage or current limits

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Motor not moving | Power/CAN issue | Check voltage and connections |
| Motor oscillating | High gains | Reduce Kp and Kd values |
| CAN communication failed | Baud rate mismatch | Verify 500Kbps setting |
| Over-temperature | High load | Reduce load or duty cycle |
| UART no response | Wrong pins | Check PB7(RX)/PA9(TX) |
| CRC error | Signal noise | Check cable shielding |

---

## Development Tools

- **STM32CubeMX** v6.15.0 - Configuration tool
- **Keil MDK-ARM** V5.32+ - ARM compiler
- **IAR EWARM** - Alternative toolchain
- **ST-Link** - Debug probe

---

## References

- [EL05 Motor User Manual](Motor_Drivers/EL05_MOTOR_DRIVE/EL05电机驱动使用指南.md)
- [NRF24L01+ Wireless Module Driver](Motor_Drivers/NRF24L01_DRIVER/README.md)
- [STM32F407 Reference Manual (RM0090)](https://www.st.com/resource/en/reference_manual/dm00031051.pdf)
- [CAN Protocol Specification](https://www.can-cia.org/)

---

## License

This project is developed for educational purposes as part of the 2026 Mingyue Class project.

---

## Contributors

- **Mingyue Class 2026** - Wheel-legged robot development team

---

<br>

---

# 中文文档

[English](#overview) | **中文**

---

## 项目概述

本项目实现了轮足机器人的嵌入式控制系统，专为**2026年明月班**设计。系统基于STM32F407IGHx微控制器，通过CAN和UART通信接口支持多种电机类型。

### 主要特性

- **多电机支持**：EL05（CAN扩展帧）、M1502E（CAN标准帧）、M0601C（UART）
- **多种控制模式**：MIT模式、位置控制、速度控制、电流控制
- **实时通信**：CAN 2.0 @ 500Kbps，UART @ 115200bps（DMA模式）
- **STM32 HAL框架**：基于STM32CubeMX生成的代码
- **模块化架构**：独立驱动模块，易于集成

---

## 硬件要求

### 主控制器

| 组件 | 规格 |
|------|------|
| MCU | STM32F407IGHx (UFBGA176封装) |
| 内核 | ARM Cortex-M4 @ 168 MHz，带FPU |
| CAN接口 | CAN1 (PD0: RX, PD1: TX) |
| UART接口 | USART1 (PB7: RX, PA9: TX)，带DMA |
| 调试接口 | SWD (PA13: SWDIO, PA14: SWCLK) |
| 外部晶振 | 12 MHz HSE |

### 支持的电机和模块

#### NRF24L01+ 无线模块 (遥控器)

| 参数 | 数值 |
|------|------|
| 工作电压 | 1.9V - 3.6V (推荐3.3V) |
| 频率范围 | 2.400GHz - 2.525GHz |
| 数据速率 | 250kbps (已配置) |
| 发射功率 | -6dBm (已配置) |
| 通信方式 | SPI接口 (SPI3) |
| 通信距离 | 开阔地最远100米 |
| 特性 | 自动应答、自动重传、对码协议 |

**硬件连接：**
```
NRF24L01模块        STM32F407
─────────────────────────────
VCC         ───►   3.3V (⚠️ 不要接5V!)
GND         ───►   GND
CE          ───►   PC8
CSN         ───►   PC9
SCK         ───►   PC10 (SPI3_SCK)
MOSI        ───►   PC12 (SPI3_MOSI)
MISO        ───►   PC11 (SPI3_MISO)
IRQ         ───►   PC7 (可选)
```

**对码协议：**
- 默认对码地址：`"HXFB0"`
- 对码后新地址：`"HXFB1"`
- RF通道：25 (2.425GHz)
- 负载宽度：16字节
- 自动重传：8次重试，2000µs延时

#### EL05 准直驱电机（CAN扩展帧）

| 参数 | 数值 |
|------|------|
| 额定电压 | 48V DC（15V-60V范围） |
| 额定扭矩 | 1.8 N·m |
| 峰值扭矩 | 6 N·m（持续5秒） |
| 空载转速 | 430 rpm |
| 减速比 | 9:1 |
| 通信方式 | CAN 2.0扩展帧 @ 500Kbps |
| 控制模式 | MIT、位置(PP/CSP)、速度、电流 |

#### M1502E 电机（CAN标准帧）

| 参数 | 数值 |
|------|------|
| 通信方式 | CAN 2.0标准帧 @ 500Kbps |
| 控制模式 | 速度模式 |
| ID范围 | 1-4（可通过CAN配置） |

#### M0601C 电机（UART）

| 参数 | 数值 |
|------|------|
| 通信方式 | UART @ 115200 bps, 8N1 |
| 帧长度 | 10字节（含CRC-8/MAXIM校验） |
| 控制模式 | 电流环、速度环、位置环 |
| ID范围 | 1-4 |

---

## 项目结构

```
Wheel-Legged_Robot/
├── Core/                              # 主应用代码
│   ├── Inc/                           # 头文件
│   │   ├── main.h
│   │   ├── can.h
│   │   └── gpio.h
│   └── Src/                           # 源文件
│       ├── main.c                     # 主程序入口
│       ├── can.c                      # CAN初始化及M1502E驱动
│       └── gpio.c                     # GPIO配置
│
├── Motor_Drivers/                     # 电机驱动模块
│   ├── EL05_MOTOR_DRIVE/              # EL05 CAN电机驱动
│   │   ├── Core/
│   │   │   ├── Inc/
│   │   │   │   ├── el05_motor.h       # EL05驱动API
│   │   │   │   └── can_rx_handler.h   # CAN接收处理
│   │   │   └── Src/
│   │   │       ├── el05_motor.c       # EL05实现
│   │   │       ├── can_rx_handler.c   # 反馈解析
│   │   │       └── main_example.c     # 使用示例
│   │   └── EL05电机驱动使用指南.md     # 中文文档
│   │
│   └── M0601C_DRIVE/                  # M0601C UART电机驱动
│       ├── Core/
│       │   ├── Inc/
│       │   │   └── motor_driver.h     # M0601C驱动API
│       │   └── Src/
│       │       └── motor_driver.c     # M0601C实现
│       └── *.md                       # 文档文件
│
├── Drivers/                           # STM32 HAL及CMSIS库
├── MDK-ARM/                           # Keil MDK工程文件
├── EWARM/                             # IAR EWARM工程文件
└── WheelRobot.ioc                     # STM32CubeMX配置文件
```

---

## 快速开始

### 1. 硬件连接

**EL05电机（CAN扩展帧）：**
```
EL05电机            STM32F407
─────────────────────────────
CAN_H        ───►   CAN_H (PD1)
CAN_L        ───►   CAN_L (PD0)
GND          ───►   GND
48V DC       ───►   电源 (15V-60V)

注意：CAN总线两端需加120Ω终端电阻
```

**M1502E电机（CAN标准帧）：**
```
M1502E电机          STM32F407
─────────────────────────────
CAN_H        ───►   CAN_H (PD1)
CAN_L        ───►   CAN_L (PD0)
GND          ───►   GND

注意：与EL05共用同一条CAN总线
```

**M0601C电机（UART）：**
```
M0601C电机          STM32F407
─────────────────────────────
TX           ───►   USART1_RX (PB7)
RX           ───►   USART1_TX (PA9)
GND          ───►   GND
```

**NRF24L01+无线模块：**
```
NRF24L01模块        STM32F407
─────────────────────────────
VCC         ───►   3.3V (⚠️ 不要接5V!)
GND         ───►   GND
CE          ───►   PC8
CSN         ───►   PC9
SCK         ───►   PC10 (SPI3_SCK)
MOSI        ───►   PC12 (SPI3_MOSI)
MISO        ───►   PC11 (SPI3_MISO)
IRQ         ───►   PC7 (可选)
```

### 2. 软件编译

#### 使用Keil MDK-ARM

1. 打开 `MDK-ARM/WheelRobot.uvprojx`
2. 选择目标配置
3. 编译工程（F7）
4. 下载到目标板（F8）

#### 使用IAR EWARM

1. 打开 `EWARM/WheelRobot.eww`
2. 编译并下载

### 3. 代码集成

**EL05电机示例：**

```c
#include "el05_motor.h"

EL05_MotorHandle_t motor1;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();

    // 初始化EL05驱动
    EL05_Init(&hcan1);
    EL05_StartReception();

    // 配置电机
    motor1.can_id = 0x01;
    EL05_Enable(&motor1);
    HAL_Delay(100);

    // MIT模式控制
    EL05_MitControl_t cmd = {
        .p_des = 0.0f,   // 目标位置 (rad)
        .v_des = 1.0f,   // 目标速度 (rad/s)
        .kp = 10.0f,     // 位置增益
        .kd = 0.5f,      // 速度增益
        .t_ff = 0.0f     // 前馈力矩 (N·m)
    };

    while (1) {
        EL05_MitControl(&motor1, &cmd);
        HAL_Delay(10);  // 100Hz控制频率
    }
}
```

**M1502E电机示例：**

```c
#include "can.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();

    // 切换到速度模式
    M1502E_SetSpeedMode();
    HAL_Delay(100);

    // 设置电机转速 (RPM)
    M1502E_SetVelocity(30);  // 30 RPM

    while (1) {
        // 主循环
    }
}
```

**M0601C电机示例：**

```c
#include "motor_driver.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    // 启动电机反馈接收
    MOTOR_StartReceive();

    // 设置电机速度（电机ID 1，30 RPM）
    MOTOR_SetSpeed(1, 30);

    while (1) {
        // 获取电机状态
        MotorStatus_t *status = MOTOR_GetStatus();
        // 使用 status->speed, status->current 等
        HAL_Delay(100);
    }
}
```

---

## API参考

### EL05电机驱动（CAN扩展帧）

| 函数 | 描述 |
|------|------|
| `EL05_Init(hcan)` | 使用CAN句柄初始化驱动 |
| `EL05_StartReception()` | 启动CAN接收中断 |
| `EL05_Enable(motor)` | 使能电机 |
| `EL05_Disable(motor)` | 停止电机 |
| `EL05_SetMode(motor, mode)` | 设置控制模式 |
| `EL05_MitControl(motor, cmd)` | MIT模式控制 |
| `EL05_PositionControl(motor, pos, vel_limit)` | 位置控制 |
| `EL05_VelocityControl(motor, vel, cur_limit)` | 速度控制 |
| `EL05_CurrentControl(motor, cur)` | 电流控制 |
| `EL05_SetZeroPosition(motor)` | 设置机械零点 |
| `EL05_WriteParam(motor, addr, value)` | 写参数 |
| `EL05_ReadParam(motor, addr)` | 读参数 |
| `EL05_GetFeedback(motor)` | 获取反馈数据 |
| `EL05_CheckOnline(motor, timeout)` | 检查电机在线状态 |

### M1502E电机驱动（CAN标准帧）

| 函数 | 描述 |
|------|------|
| `M1502E_SetSpeedMode()` | 切换电机到速度模式 |
| `M1502E_SetVelocity(rpm)` | 设置电机转速 (RPM) |
| `M1502E_Config_ID(id)` | 设置电机ID (1-8) |

### M0601C电机驱动（UART）

| 函数 | 描述 |
|------|------|
| `MOTOR_CRC8_Calc(data, len)` | 计算CRC-8/MAXIM |
| `MOTOR_SendDriveCmd(id, value, acc, brake)` | 发送驱动指令 |
| `MOTOR_SetSpeed(id, rpm)` | 设置电机速度 |
| `MOTOR_Brake(id)` | 电机刹车 |
| `MOTOR_Stop(id)` | 电机停止 |
| `MOTOR_SendModeSwitchCmd(id, mode)` | 切换控制模式 |
| `MOTOR_SendSetIDCmd(newId)` | 设置电机ID |
| `MOTOR_StartReceive()` | 启动UART DMA接收 |
| `MOTOR_GetFeedback()` | 获取原始反馈 |
| `MOTOR_GetStatus()` | 获取解析后状态 |

---

## 控制模式

### EL05 MIT模式（运控模式）

最灵活的控制模式，适用于机器人应用：

```
输出力矩 = Kp × (p_des - p_actual) + Kd × (v_des - v_actual) + t_ff
```

**参数范围：**

| 参数 | 范围 | 单位 |
|------|------|------|
| p_des | -12.5 ~ 12.5 | rad |
| v_des | -30 ~ 30 | rad/s |
| kp | 0 ~ 500 | - |
| kd | 0 ~ 5 | - |
| t_ff | -18 ~ 18 | N·m |

---

## 安全注意事项

### EL05电机

- **工作电压**：15V - 60V DC（额定：48V）
- **最大扭矩持续时间**：

| 扭矩 (N·m) | 持续时间 (s) |
|------------|--------------|
| 6 | 5 |
| 5 | 7 |
| 4 | 14 |
| 3 | 44 |
| 2 | 300 |
| 1.8 | 连续 |

- **温度保护**：145°C自动停机

### 通用安全

- 上电前务必检查电机连接
- 确保CAN终端电阻正确（两端各120Ω）
- 运行时监控电机温度
- 从低增益开始，逐渐增加
- 切勿超过额定电压或电流限制

---

## 故障排除

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 电机不转 | 电源/CAN问题 | 检查电压和连接 |
| 电机抖动 | 增益过高 | 降低Kp和Kd值 |
| CAN通信失败 | 波特率不匹配 | 确认500Kbps设置 |
| 过热 | 负载过大 | 降低负载或占空比 |
| UART无响应 | 引脚错误 | 检查PB7(RX)/PA9(TX) |
| CRC错误 | 信号干扰 | 检查线缆屏蔽 |

---

## 开发工具

- **STM32CubeMX** v6.15.0 - 配置工具
- **Keil MDK-ARM** V5.32+ - ARM编译器
- **IAR EWARM** - 备选工具链
- **ST-Link** - 调试探针

---

## 参考资料

- [EL05电机使用手册](Motor_Drivers/EL05_MOTOR_DRIVE/EL05电机驱动使用指南.md)
- [NRF24L01+ 无线模块驱动](Motor_Drivers/NRF24L01_DRIVER/README.md)
- [STM32F407参考手册 (RM0090)](https://www.st.com/resource/en/reference_manual/dm00031051.pdf)
- [CAN协议规范](https://www.can-cia.org/)

---

## 许可证

本项目为2026年明月班项目的教育目的而开发。

---

## 贡献者

- **明月班2026** - 轮足机器人开发团队

---

**Happy Coding! | 祝开发顺利！**
