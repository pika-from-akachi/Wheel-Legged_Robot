# NRF24L01+ 2.4G无线模块驱动 | NRF24L01+ 2.4G Wireless Module Driver

[![Platform](https://img.shields.io/badge/Platform-STM32F407-blue.svg)](https://www.st.com)
[![Framework](https://img.shields.io/badge/Framework-STM32_HAL-green.svg)](https://www.st.com/en/embedded-software/stm32cube.html)
[![Language](https://img.shields.io/badge/Language-C-orange.svg)](https://en.wikipedia.org/wiki/C_(programming_language))

**English** | [中文](#中文文档)

---

## Overview

This driver provides a complete implementation for the NRF24L01+ 2.4GHz wireless transceiver module, designed for the wheel-legged robot project. It supports both transmitter (remote controller) and receiver (robot) modes with reliable communication.

### Key Features

- **STM32 HAL Compatible**: Built on STM32 HAL library
- **Flexible Configuration**: Adjustable RF channel, data rate, TX power
- **Auto-Acknowledgment**: Reliable data transmission with auto-retransmit
- **Multiple Data Rates**: 250kbps, 1Mbps, 2Mbps
- **Variable Payload**: 1-32 bytes payload width
- **Bidirectional Communication**: Switchable TX/RX modes
- **Interrupt Support**: Optional IRQ pin for event-driven operation

---

## Hardware Requirements

### NRF24L01+ Module Specifications

| Parameter | Value |
|-----------|-------|
| Operating Voltage | 1.9V - 3.6V (3.3V recommended) |
| Operating Current | 13.5mA (TX at 0dBm) |
| Standby Current | 22µA |
| Frequency Range | 2.400GHz - 2.525GHz |
| Data Rate | 250kbps / 1Mbps / 2Mbps |
| TX Power | -18dBm / -12dBm / -6dBm / 0dBm |
| Range | Up to 100m (open area) |

### Hardware Connection

**STM32F407 Pin Configuration:**

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

**⚠️ Important Notes:**

1. **Power Supply**: NRF24L01+ operates at 3.3V. Connecting to 5V will damage the module!
2. **SPI Configuration**: Using SPI3 in master mode, up to 10MHz clock
3. **GPIO Levels**: STM32F407 GPIOs are 5V tolerant but NRF24L01+ is not
4. **Decoupling**: Add 10µF and 100nF capacitors near VCC/GND pins

---

## Quick Start

### 1. SPI Configuration (STM32CubeMX)

Configure SPI3 in STM32CubeMX:

```
Mode: Full-Duplex Master
Clock: Up to 10MHz (Prescaler: 16)
CPOL: Low
CPHA: 1 Edge
NSS: Software controlled

Pinout:
- PC10: SPI3_SCK
- PC11: SPI3_MISO
- PC12: SPI3_MOSI
```

### 2. GPIO Configuration

Configure CE (PC8) and CSN (PC9) pins as GPIO outputs:

```c
// In main.c or gpio.c
// Or use nrf24l01_hal_config.h helper function
NRF24L01_GPIO_Init(); // Initializes PC8, PC9, PC7

// Manual configuration:
GPIO_InitTypeDef GPIO_InitStruct = {0};

// CE Pin (PC8)
GPIO_InitStruct.Pin = GPIO_PIN_8;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

// CSN Pin (PC9)
GPIO_InitStruct.Pin = GPIO_PIN_9;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
```

### 3. Initialization

**Transmitter (Remote Controller):**

```c
#include "nrf24l01.h"
#include "nrf24l01_hal_config.h"

NRF24L01_Handle_t nrf_handle;

void main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI3_Init(); // Use SPI3

    // Initialize GPIO pins
    NRF24L01_GPIO_Init();

    NRF24L01_Init_t config = {
        .hspi = &hspi3,                     // SPI3 handle
        .ce_port = NRF24L01_CE_PORT,        // PC8
        .ce_pin = NRF24L01_CE_PIN,
        .csn_port = NRF24L01_CSN_PORT,      // PC9
        .csn_pin = NRF24L01_CSN_PIN,
        .irq_port = NRF24L01_IRQ_PORT,      // PC7
        .irq_pin = NRF24L01_IRQ_PIN,
        .channel = 70,
        .payload_width = 16,
        .data_rate = NRF24L01_RF_DR_2MBPS,
        .tx_power = NRF24L01_RF_PWR_0DBM,
        .address = {0x34, 0x43, 0x10, 0x10, 0x01},
        .auto_ack = true,
        .dynamic_payload = false
    };

    NRF24L01_Init(&nrf_handle, &config);
    NRF24L01_SetTxMode(&nrf_handle);

    while (1) {
        uint8_t data[16] = {0xAA, 0x01, 0x02, 0x03, 0x55, 0xFF};
        NRF24L01_Transmit(&nrf_handle, data, 16);
        HAL_Delay(50);
    }
}
```

**Receiver (Robot):**

```c
#include "nrf24l01.h"
#include "nrf24l01_hal_config.h"

NRF24L01_Handle_t nrf_handle;

void main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI3_Init(); // Use SPI3

    // Initialize GPIO pins
    NRF24L01_GPIO_Init();

    NRF24L01_Init_t config = {
        .hspi = &hspi3,                     // SPI3 handle
        .ce_port = NRF24L01_CE_PORT,        // PC8
        .ce_pin = NRF24L01_CE_PIN,
        .csn_port = NRF24L01_CSN_PORT,      // PC9
        .csn_pin = NRF24L01_CSN_PIN,
        .irq_port = NRF24L01_IRQ_PORT,      // PC7
        .irq_pin = NRF24L01_IRQ_PIN,
        .channel = 70,
        .payload_width = 16,
        .data_rate = NRF24L01_RF_DR_2MBPS,
        .tx_power = NRF24L01_RF_PWR_0DBM,
        .address = {0x34, 0x43, 0x10, 0x10, 0x01},
        .auto_ack = true,
        .dynamic_payload = false
    };

    NRF24L01_Init(&nrf_handle, &config);
    NRF24L01_SetRxMode(&nrf_handle);

    while (1) {
        if (NRF24L01_DataReady(&nrf_handle)) {
            uint8_t rx_data[16];
            uint8_t length;
            NRF24L01_Receive(&nrf_handle, rx_data, &length);
            // Process received data...
        }
        HAL_Delay(10);
    }
}
```

---

## API Reference

### Initialization Functions

| Function | Description |
|----------|-------------|
| `NRF24L01_Init(handle, config)` | Initialize module with configuration |
| `NRF24L01_DeInit(handle)` | De-initialize module |

### Configuration Functions

| Function | Description |
|----------|-------------|
| `NRF24L01_SetChannel(handle, channel)` | Set RF channel (0-125) |
| `NRF24L01_SetDataRate(handle, rate)` | Set data rate (250kbps/1Mbps/2Mbps) |
| `NRF24L01_SetTxPower(handle, power)` | Set TX power (-18/-12/-6/0 dBm) |
| `NRF24L01_SetPayloadWidth(handle, pipe, width)` | Set payload width (1-32 bytes) |
| `NRF24L01_SetAddress(handle, reg, addr, width)` | Set TX/RX address |

### Mode Control Functions

| Function | Description |
|----------|-------------|
| `NRF24L01_SetTxMode(handle)` | Switch to transmitter mode |
| `NRF24L01_SetRxMode(handle)` | Switch to receiver mode |
| `NRF24L01_PowerDown(handle)` | Power down module |
| `NRF24L01_PowerUp(handle)` | Power up module |

### Data Transmission Functions

| Function | Description |
|----------|-------------|
| `NRF24L01_Transmit(handle, data, length)` | Transmit data packet |
| `NRF24L01_TransmitNoAck(handle, data, length)` | Transmit without auto-ack |
| `NRF24L01_Receive(handle, data, length)` | Receive data packet |

### Status Functions

| Function | Description |
|----------|-------------|
| `NRF24L01_GetStatus(handle)` | Get module status |
| `NRF24L01_GetFifoStatus(handle)` | Get FIFO status |
| `NRF24L01_DataReady(handle)` | Check if RX data available |
| `NRF24L01_IsSending(handle)` | Check if TX in progress |

### FIFO Control Functions

| Function | Description |
|----------|-------------|
| `NRF24L01_FlushTx(handle)` | Flush TX FIFO |
| `NRF24L01_FlushRx(handle)` | Flush RX FIFO |

---

## Integration with Motor Drivers

### Example: Control EL05 Motors

```c
#include "nrf24l01.h"
#include "el05_motor.h"

void RemoteControlTask(void)
{
    int8_t left_speed, right_speed;
    uint8_t button;

    if (NRF24L01_ReceiveControlData(&left_speed, &right_speed, &button)) {
        // Convert to rad/s
        float left_vel = left_speed * 0.1f;
        float right_vel = right_speed * 0.1f;

        // Control EL05 motors
        EL05_MotorHandle_t motor_left = {.can_id = 0x01};
        EL05_MotorHandle_t motor_right = {.can_id = 0x02};

        EL05_VelocityControl(&motor_left, left_vel, 5.0f);
        EL05_VelocityControl(&motor_right, right_vel, 5.0f);
    }
}
```

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| No communication | Wrong SPI mode | Check CPOL/CPHA settings |
| Short range | Low TX power | Increase TX power setting |
| Data loss | No auto-ack | Enable auto-acknowledgment |
| Module not responding | Wrong power supply | Verify 3.3V supply |
| CRC errors | Signal interference | Change RF channel |
| High latency | Low data rate | Use 2Mbps data rate |

---

## References

- [NRF24L01+ Datasheet](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF24-series)
- [STM32F407 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031051.pdf)
- [EL05 Motor Driver](../EL05_MOTOR_DRIVE/)

---

<br>

---

# 中文文档

[English](#overview) | **中文**

---

## 概述

本驱动为轮足机器人项目提供NRF24L01+ 2.4GHz无线收发模块的完整实现,支持发射器(遥控器)和接收器(机器人)模式,实现可靠通信。

### 主要特性

- **STM32 HAL兼容**: 基于STM32 HAL库开发
- **灵活配置**: 可调节RF通道、数据速率、发射功率
- **自动应答**: 可靠的数据传输与自动重传
- **多种数据速率**: 250kbps、1Mbps、2Mbps
- **可变载荷**: 1-32字节载荷宽度
- **双向通信**: 可切换TX/RX模式
- **中断支持**: 可选IRQ引脚实现事件驱动

---

## 硬件要求

### NRF24L01+ 模块规格

| 参数 | 数值 |
|------|------|
| 工作电压 | 1.9V - 3.6V (推荐3.3V) |
| 工作电流 | 13.5mA (0dBm发射) |
| 待机电流 | 22µA |
| 频率范围 | 2.400GHz - 2.525GHz |
| 数据速率 | 250kbps / 1Mbps / 2Mbps |
| 发射功率 | -18dBm / -12dBm / -6dBm / 0dBm |
| 通信距离 | 开阔地最远100米 |

### 硬件连接

**STM32F407 引脚配置:**

```
NRF24L01模块         STM32F407
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

**⚠️ 重要提示:**

1. **电源**: NRF24L01+工作电压为3.3V,接5V会烧毁模块!
2. **SPI配置**: 使用SPI3主机模式,时钟最高10MHz
3. **GPIO电平**: STM32F407的GPIO兼容5V,但NRF24L01+不兼容
4. **去耦**: 在VCC/GND引脚附近添加10µF和100nF电容

---

## 快速开始

### 1. SPI配置 (STM32CubeMX)

在STM32CubeMX中配置SPI3:

```
模式: 全双工主机
时钟: 最高10MHz (预分频: 16)
CPOL: Low
CPHA: 1 Edge
NSS: 软件控制

引脚分配:
- PC10: SPI3_SCK
- PC11: SPI3_MISO
- PC12: SPI3_MOSI
```

### 2. GPIO配置

配置CE (PC8) 和CSN (PC9) 引脚为GPIO输出:

```c
// 在main.c或gpio.c中
// 或使用 nrf24l01_hal_config.h 辅助函数
NRF24L01_GPIO_Init(); // 初始化 PC8, PC9, PC7

// 手动配置:
GPIO_InitTypeDef GPIO_InitStruct = {0};

// CE引脚 (PC8)
GPIO_InitStruct.Pin = GPIO_PIN_8;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

// CSN引脚 (PC9)
GPIO_InitStruct.Pin = GPIO_PIN_9;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
```

### 3. 初始化

**发射器(遥控器):**

```c
#include "nrf24l01.h"
#include "nrf24l01_hal_config.h"

NRF24L01_Handle_t nrf_handle;

void main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI3_Init(); // 使用SPI3

    // 初始化GPIO引脚
    NRF24L01_GPIO_Init();

    NRF24L01_Init_t config = {
        .hspi = &hspi3,                     // SPI3句柄
        .ce_port = NRF24L01_CE_PORT,        // PC8
        .ce_pin = NRF24L01_CE_PIN,
        .csn_port = NRF24L01_CSN_PORT,      // PC9
        .csn_pin = NRF24L01_CSN_PIN,
        .irq_port = NRF24L01_IRQ_PORT,      // PC7
        .irq_pin = NRF24L01_IRQ_PIN,
        .channel = 70,
        .payload_width = 16,
        .data_rate = NRF24L01_RF_DR_2MBPS,
        .tx_power = NRF24L01_RF_PWR_0DBM,
        .address = {0x34, 0x43, 0x10, 0x10, 0x01},
        .auto_ack = true,
        .dynamic_payload = false
    };

    NRF24L01_Init(&nrf_handle, &config);
    NRF24L01_SetTxMode(&nrf_handle);

    while (1) {
        uint8_t data[16] = {0xAA, 0x01, 0x02, 0x03, 0x55, 0xFF};
        NRF24L01_Transmit(&nrf_handle, data, 16);
        HAL_Delay(50);
    }
}
```

**接收器(机器人):**

```c
#include "nrf24l01.h"
#include "nrf24l01_hal_config.h"

NRF24L01_Handle_t nrf_handle;

void main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI3_Init(); // 使用SPI3

    // 初始化GPIO引脚
    NRF24L01_GPIO_Init();

    NRF24L01_Init_t config = {
        .hspi = &hspi3,                     // SPI3句柄
        .ce_port = NRF24L01_CE_PORT,        // PC8
        .ce_pin = NRF24L01_CE_PIN,
        .csn_port = NRF24L01_CSN_PORT,      // PC9
        .csn_pin = NRF24L01_CSN_PIN,
        .irq_port = NRF24L01_IRQ_PORT,      // PC7
        .irq_pin = NRF24L01_IRQ_PIN,
        .channel = 70,
        .payload_width = 16,
        .data_rate = NRF24L01_RF_DR_2MBPS,
        .tx_power = NRF24L01_RF_PWR_0DBM,
        .address = {0x34, 0x43, 0x10, 0x10, 0x01},
        .auto_ack = true,
        .dynamic_payload = false
    };

    NRF24L01_Init(&nrf_handle, &config);
    NRF24L01_SetRxMode(&nrf_handle);

    while (1) {
        if (NRF24L01_DataReady(&nrf_handle)) {
            uint8_t rx_data[16];
            uint8_t length;
            NRF24L01_Receive(&nrf_handle, rx_data, &length);
            // 处理接收到的数据...
        }
        HAL_Delay(10);
    }
}
```

---

## API参考

### 初始化函数

| 函数 | 描述 |
|------|------|
| `NRF24L01_Init(handle, config)` | 使用配置初始化模块 |
| `NRF24L01_DeInit(handle)` | 反初始化模块 |

### 配置函数

| 函数 | 描述 |
|------|------|
| `NRF24L01_SetChannel(handle, channel)` | 设置RF通道 (0-125) |
| `NRF24L01_SetDataRate(handle, rate)` | 设置数据速率 (250kbps/1Mbps/2Mbps) |
| `NRF24L01_SetTxPower(handle, power)` | 设置发射功率 (-18/-12/-6/0 dBm) |
| `NRF24L01_SetPayloadWidth(handle, pipe, width)` | 设置载荷宽度 (1-32字节) |
| `NRF24L01_SetAddress(handle, reg, addr, width)` | 设置TX/RX地址 |

### 模式控制函数

| 函数 | 描述 |
|------|------|
| `NRF24L01_SetTxMode(handle)` | 切换到发射模式 |
| `NRF24L01_SetRxMode(handle)` | 切换到接收模式 |
| `NRF24L01_PowerDown(handle)` | 模块掉电 |
| `NRF24L01_PowerUp(handle)` | 模块上电 |

### 数据传输函数

| 函数 | 描述 |
|------|------|
| `NRF24L01_Transmit(handle, data, length)` | 发送数据包 |
| `NRF24L01_TransmitNoAck(handle, data, length)` | 发送数据包(无应答) |
| `NRF24L01_Receive(handle, data, length)` | 接收数据包 |

### 状态函数

| 函数 | 描述 |
|------|------|
| `NRF24L01_GetStatus(handle)` | 获取模块状态 |
| `NRF24L01_GetFifoStatus(handle)` | 获取FIFO状态 |
| `NRF24L01_DataReady(handle)` | 检查是否有接收数据 |
| `NRF24L01_IsSending(handle)` | 检查是否正在发送 |

### FIFO控制函数

| 函数 | 描述 |
|------|------|
| `NRF24L01_FlushTx(handle)` | 清空TX FIFO |
| `NRF24L01_FlushRx(handle)` | 清空RX FIFO |

---

## 与电机驱动集成

### 示例: 控制EL05电机

```c
#include "nrf24l01.h"
#include "el05_motor.h"

void RemoteControlTask(void)
{
    int8_t left_speed, right_speed;
    uint8_t button;

    if (NRF24L01_ReceiveControlData(&left_speed, &right_speed, &button)) {
        // 转换为rad/s
        float left_vel = left_speed * 0.1f;
        float right_vel = right_speed * 0.1f;

        // 控制EL05电机
        EL05_MotorHandle_t motor_left = {.can_id = 0x01};
        EL05_MotorHandle_t motor_right = {.can_id = 0x02};

        EL05_VelocityControl(&motor_left, left_vel, 5.0f);
        EL05_VelocityControl(&motor_right, right_vel, 5.0f);
    }
}
```

---

## 故障排除

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 无法通信 | SPI模式错误 | 检查CPOL/CPHA设置 |
| 距离短 | 发射功率低 | 提高发射功率设置 |
| 数据丢失 | 无自动应答 | 启用自动应答 |
| 模块无响应 | 电源错误 | 确认3.3V供电 |
| CRC错误 | 信号干扰 | 更换RF通道 |
| 延迟高 | 数据速率低 | 使用2Mbps数据速率 |

---

## 参考资料

- [NRF24L01+ 数据手册](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF24-series)
- [STM32F407 参考手册](https://www.st.com/resource/en/reference_manual/dm00031051.pdf)
- [EL05电机驱动](../EL05_MOTOR_DRIVE/)

---

**Happy Coding! | 祝开发顺利!**
