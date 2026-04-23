# Wheel-Legged Robot Control System | 轮足机器人控制系统

[![Platform](https://img.shields.io/badge/Platform-STM32F407-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407-417.html)
[![Language](https://img.shields.io/badge/Language-C-orange.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

**English** | [中文](#中文文档)

---

## Overview

This project implements the embedded control system for a wheel-legged robot, designed for the 2026 Mingyue Class. The system is based on STM32F407IGHx microcontroller and supports multiple motor control modes through CAN and UART communication interfaces.

### Key Features

- **Dual Motor Driver Support**: EL05 (CAN) and M0601C (UART) motor drivers
- **Multiple Control Modes**: MIT mode, Position, Velocity, and Current control
- **Real-time Communication**: CAN 2.0 extended frames @ 500Kbps, UART @ 115200bps
- **STM32 HAL Framework**: Built with STM32CubeMX generated code
- **Modular Architecture**: Easy to extend and maintain

---

## Hardware Requirements

### Main Controller

| Component | Specification |
|-----------|---------------|
| MCU | STM32F407IGHx (UFBGA176 package) |
| System Clock | 168 MHz |
| CAN Interface | CAN1 (PD0: RX, PD1: TX) |
| Debug Interface | SWD (PA13: SWDIO, PA14: SWCLK) |
| External Oscillator | 12 MHz HSE |

### Supported Motors

#### EL05 Quasi-Direct-Drive Motor (CAN)

| Parameter | Value |
|-----------|-------|
| Rated Voltage | 48V DC |
| Rated Torque | 1.8 N·m |
| Peak Torque | 6 N·m |
| No-load Speed | 430 rpm |
| Gear Ratio | 9:1 |
| Communication | CAN 2.0 Extended Frame @ 1Mbps |

#### M0601C Motor (UART)

| Parameter | Value |
|-----------|-------|
| Communication | UART @ 115200 bps |
| Frame Length | 10 bytes (with CRC-8/MAXIM) |
| Control Modes | Current, Speed, Position |

---

## Project Structure

```
Wheel-Legged_Robot/
├── Core/                          # Main application code
│   ├── Inc/                       # Header files
│   │   ├── main.h
│   │   ├── can.h
│   │   └── gpio.h
│   └── Src/                       # Source files
│       ├── main.c                 # Main entry point
│       ├── can.c                  # CAN initialization & M1502E driver
│       └── gpio.c                 # GPIO configuration
│
├── Motor_Drivers/                 # Motor driver modules
│   ├── EL05_MOTOR_DRIVE/          # EL05 CAN motor driver
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
│   └── M0601C_DRIVE/              # M0601C UART motor driver
│       ├── Core/
│       │   ├── Inc/
│       │   │   └── motor_driver.h     # M0601C driver API
│       │   └── Src/
│       │       └── motor_driver.c     # M0601C implementation
│       └── *.md                       # Documentation files
│
├── Drivers/                       # STM32 HAL & CMSIS libraries
├── MDK-ARM/                       # Keil MDK project files
├── EWARM/                         # IAR EWARM project files
└── WheelRobot.ioc                 # STM32CubeMX configuration
```

---

## Quick Start

### 1. Hardware Setup

**EL05 Motor Connection:**
```
EL05 Motor          STM32F407
─────────────────────────────
CAN_H        ───►   CAN_H (PD1)
CAN_L        ───►   CAN_L (PD0)
GND          ───►   GND
48V DC       ───►   Power Supply (15V-60V)

Note: Add 120Ω termination resistor at both ends of CAN bus
```

**M0601C Motor Connection:**
```
M0601C Motor        STM32F407
─────────────────────────────
TX           ───►   USART1_RX (PA10)
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

// Motor handle
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

### EL05 Motor Driver (CAN)

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
| `EL05_GetFeedback(motor)` | Get feedback data |
| `EL05_CheckOnline(motor, timeout)` | Check motor online status |

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
- Ensure proper CAN termination (120Ω)
- Monitor motor temperature during operation
- Start with low gains and increase gradually

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Motor not moving | Power/CAN issue | Check voltage and connections |
| Motor oscillating | High gains | Reduce Kp and Kd values |
| CAN communication failed | Baud rate mismatch | Verify 500Kbps setting |
| Over-temperature | High load | Reduce load or duty cycle |

---

## Development Tools

- **STM32CubeMX** v6.15.0 - Configuration tool
- **Keil MDK-ARM** V5.32+ - ARM compiler
- **IAR EWARM** - Alternative toolchain
- **ST-Link** - Debug probe

---

## References

- [EL05 Motor User Manual](Motor_Drivers/EL05_MOTOR_DRIVE/EL05电机驱动使用指南.md)
- [STM32F407 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031051.pdf)
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

本项目实现了轮足机器人的嵌入式控制系统，专为2026年明月班设计。系统基于STM32F407IGHx微控制器，通过CAN和UART通信接口支持多种电机控制模式。

### 主要特性

- **双电机驱动支持**：EL05（CAN）和M0601C（UART）电机驱动器
- **多种控制模式**：MIT模式、位置控制、速度控制、电流控制
- **实时通信**：CAN 2.0扩展帧 @ 500Kbps，UART @ 115200bps
- **STM32 HAL框架**：基于STM32CubeMX生成的代码
- **模块化架构**：易于扩展和维护

---

## 硬件要求

### 主控制器

| 组件 | 规格 |
|------|------|
| MCU | STM32F407IGHx (UFBGA176封装) |
| 系统时钟 | 168 MHz |
| CAN接口 | CAN1 (PD0: RX, PD1: TX) |
| 调试接口 | SWD (PA13: SWDIO, PA14: SWCLK) |
| 外部晶振 | 12 MHz HSE |

### 支持的电机

#### EL05 准直驱电机（CAN）

| 参数 | 数值 |
|------|------|
| 额定电压 | 48V DC |
| 额定扭矩 | 1.8 N·m |
| 峰值扭矩 | 6 N·m |
| 空载转速 | 430 rpm |
| 减速比 | 9:1 |
| 通信方式 | CAN 2.0扩展帧 @ 1Mbps |

#### M0601C 电机（UART）

| 参数 | 数值 |
|------|------|
| 通信方式 | UART @ 115200 bps |
| 帧长度 | 10字节（含CRC-8/MAXIM校验） |
| 控制模式 | 电流环、速度环、位置环 |

---

## 项目结构

```
Wheel-Legged_Robot/
├── Core/                          # 主应用代码
│   ├── Inc/                       # 头文件
│   │   ├── main.h
│   │   ├── can.h
│   │   └── gpio.h
│   └── Src/                       # 源文件
│       ├── main.c                 # 主程序入口
│       ├── can.c                  # CAN初始化及M1502E驱动
│       └── gpio.c                 # GPIO配置
│
├── Motor_Drivers/                 # 电机驱动模块
│   ├── EL05_MOTOR_DRIVE/          # EL05 CAN电机驱动
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
│   └── M0601C_DRIVE/              # M0601C UART电机驱动
│       ├── Core/
│       │   ├── Inc/
│       │   │   └── motor_driver.h     # M0601C驱动API
│       │   └── Src/
│       │       └── motor_driver.c     # M0601C实现
│       └── *.md                       # 文档文件
│
├── Drivers/                       # STM32 HAL及CMSIS库
├── MDK-ARM/                       # Keil MDK工程文件
├── EWARM/                         # IAR EWARM工程文件
└── WheelRobot.ioc                 # STM32CubeMX配置文件
```

---

## 快速开始

### 1. 硬件连接

**EL05电机连接：**
```
EL05电机            STM32F407
─────────────────────────────
CAN_H        ───►   CAN_H (PD1)
CAN_L        ───►   CAN_L (PD0)
GND          ───►   GND
48V DC       ───►   电源 (15V-60V)

注意：CAN总线两端需加120Ω终端电阻
```

**M0601C电机连接：**
```
M0601C电机          STM32F407
─────────────────────────────
TX           ───►   USART1_RX (PA10)
RX           ───►   USART1_TX (PA9)
GND          ───►   GND
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

// 电机句柄
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

### EL05电机驱动（CAN）

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
| `EL05_GetFeedback(motor)` | 获取反馈数据 |
| `EL05_CheckOnline(motor, timeout)` | 检查电机在线状态 |

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
- 确保CAN终端电阻正确（120Ω）
- 运行时监控电机温度
- 从低增益开始，逐渐增加

---

## 故障排除

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 电机不转 | 电源/CAN问题 | 检查电压和连接 |
| 电机抖动 | 增益过高 | 降低Kp和Kd值 |
| CAN通信失败 | 波特率不匹配 | 确认500Kbps设置 |
| 过热 | 负载过大 | 降低负载或占空比 |

---

## 开发工具

- **STM32CubeMX** v6.15.0 - 配置工具
- **Keil MDK-ARM** V5.32+ - ARM编译器
- **IAR EWARM** - 备选工具链
- **ST-Link** - 调试探针

---

## 参考资料

- [EL05电机使用手册](Motor_Drivers/EL05_MOTOR_DRIVE/EL05电机驱动使用指南.md)
- [STM32F407参考手册](https://www.st.com/resource/en/reference_manual/dm00031051.pdf)
- [CAN协议规范](https://www.can-cia.org/)

---

## 许可证

本项目为2026年明月班项目的教育目的而开发。

---

## 贡献者

- **明月班2026** - 轮足机器人开发团队

---

**祝开发顺利！**
