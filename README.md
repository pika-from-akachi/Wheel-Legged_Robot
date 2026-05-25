# Wheel-Legged Robot Control System | 轮足机器人控制系统

[![Platform](https://img.shields.io/badge/Platform-STM32F407-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407-417.html)
[![Framework](https://img.shields.io/badge/Framework-STM32CubeMX_6.15.0-green.svg)](https://www.st.com/en/development-tools/stm32cubemx.html)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS_V10.6.2-orange.svg)](https://www.freertos.org)
[![Language](https://img.shields.io/badge/Language-C-orange.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**English** | [中文](#中文文档)

---

## Overview

This project implements the embedded control system for a wheel-legged robot, developed for the **2026 Mingyue Class**. The system is based on the STM32F407IGHx microcontroller and supports multiple motor types through CAN and UART communication interfaces.

### Key Features

- **Real-Time Operating System**: FreeRTOS V10.6.2 with 8 priority-based tasks
- **Interrupt-Driven IMU**: ICM-42688-P read by TIM2 hardware interrupt (1 kHz, deterministic timing)
- **Multi-Motor Support**: EL05 joint motor (CAN extended frame), M0601C wheel motor (RS485)
- **LQR Control Framework**: Linear Quadratic Regulator balancing with gain scheduling
- **Dual MCU Architecture**: STM32F407 (control) + ESP32-S3 (wireless tuning bridge)
- **Wireless Tuning App**: Alpine.js + Three.js 3D visualization with real-time parameter adjustment
- **3D Visualization**: Real-time robot state monitoring via Three.js
- **IMU Sensor**: ICM-42688-P 6-axis IMU with Kalman filter (SPI1)
- **Wireless Control**: NRF24L01+ remote controller with pairing protocol (SPI3)
- **Multiple Control Modes**: MIT mode, Position, Velocity, Current control
- **Real-time Communication**: CAN 2.0 @ 1Mbps, RS485 @ 115200bps, UART @ 921600bps
- **Remote Parameter Tuning**: Wireless LQR gain adjustment via ESP32-S3 web interface

---

## Hardware Requirements

### Main Controller

| Component | Specification |
|-----------|---------------|
| MCU | STM32F407IGHx (UFBGA176 package) |
| Core | ARM Cortex-M4 @ 168 MHz with FPU |
| CAN Interface | CAN1 (PD0: CAN_RX, PD1: CAN_TX) |
| RS485 Interface | USART1 (PB6: TX, PB7: RX) with PE0 direction control |
| ESP32 Interface | USART3 (PB10: TX, PB11: RX) @ 921600 baud |
| UART Interface | USART1 (PB7: RX, PA9: TX) with DMA |
| SPI Interface | SPI1 (PA5: SCK, PA6: MISO, PA7: MOSI, PA4: CS) for IMU<br>SPI3 (PC10: SCK, PC11: MISO, PC12: MOSI) for NRF24L01 |
| Debug Interface | SWD (PA13: SWDIO, PA14: SWCLK) |
| External Oscillator | 8 MHz HSE |

### Wireless Tuning Module

| Component | Specification |
|-----------|---------------|
| MCU | ESP32-S3 |
| WiFi | 2.4GHz 802.11 b/g/n, AP mode |
| AP SSID | WheelRobot-Tuning |
| AP Password | 12345678 |
| Web Interface | Alpine.js + Three.js 3D visualization |
| Protocol | WebSocket for real-time data |
| UART | TX: GPIO43, RX: GPIO44 @ 921600 baud |

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

#### EL05 Quasi-Direct-Drive Motor (CAN Extended Frame) - Joint Motor

| Parameter | Value |
|-----------|-------|
| Rated Voltage | 48V DC (15V-60V range) |
| Rated Torque | 1.8 N·m |
| Peak Torque | 6 N·m (5s duration) |
| No-load Speed | 430 rpm |
| Gear Ratio | 9:1 |
| Communication | CAN 2.0 Extended Frame @ 1Mbps |
| Control Modes | MIT, Position (PP/CSP), Velocity, Current |
| Application | Joint motor for wheel-legged robot |

#### M0601C Motor (UART) - Wheel Motor

| Parameter | Value |
|-----------|-------|
| Communication | UART @ 115200 bps, 8N1 |
| Frame Length | 10 bytes (with CRC-8/MAXIM) |
| Control Modes | Current, Speed, Position |
| ID Range | 1-4 |
| Application | Wheel motor for wheel-legged robot |

#### ICM-42688-P 6-Axis IMU Sensor

| Parameter | Value |
|-----------|-------|
| Accelerometer Range | ±16g (configurable: ±2g, ±4g, ±8g, ±16g) |
| Gyroscope Range | ±2000dps (configurable) |
| Output Data Rate | 1kHz (configurable) |
| Communication | SPI interface (SPI1) |
| Filter | Kalman filter (default), Moving Average, Low-pass |
| Temperature Sensor | Built-in |

**Hardware Connection:**
```
ICM-42688-P Module     STM32F407
──────────────────────────────
VCC         ───►   3.3V
GND         ───►   GND
CS          ───►   PA4 (GPIO Output, software controlled)
SCLK        ───►   PA5 (SPI1_SCK)
MISO        ───►   PA6 (SPI1_MISO)
MOSI        ───►   PA7 (SPI1_MOSI)
INT1        ───►   Optional (data ready interrupt)
```

**Usage Example (interrupt-driven, 1 kHz):**
```c
#include "icm42688.h"
#include "freertos_tasks.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    // Initialize ICM42688 (called from Task_IMU context)
    ICM42688_Init();

    // IMU raw data is read by TIM2 ISR at 1 kHz using
    // register-level SPI (no HAL/SysTick dependency).
    // IMU_ISR_Handler() feeds a double buffer and notifies
    // Task_IMU via task notification.

    while (1) {
        // Processed data available via queue (IMU_Data_t)
        // or watch variables:
        float accel_x = g_imu_accel_x_filtered;  // g
        float gyro_x = g_imu_gyro_x_filtered;    // deg/s
        float temp = g_imu_temperature_c;        // °C

        HAL_Delay(10);
    }
}
```

---

## FreeRTOS Task Architecture

This project uses **FreeRTOS V10.6.2** with a comprehensive task-based architecture for real-time control.

### Task Overview

| Task | Hardware | Frequency | Priority | Stack | Description |
|------|----------|-----------|----------|-------|-------------|
| **IMU** | ICM-42688-P | 1kHz | Medium (3) | 1KB | IMU data processing (raw read by TIM2 ISR) |
| **LQR** | Control Algorithm | 1kHz | High (5) | 4KB | LQR state feedback & safety monitoring |
| **Balance** | Control Algorithm | 500Hz | High (5) | 4KB | Balance control, state estimation, complementary filter |
| **Motor** | EL05 + M0601C | 200Hz | Medium-High (4) | 2KB | Motor command processing for all 6 motors |
| **Remote** | NRF24L01+ | 100Hz | Medium (3) | 2KB | Remote controller data reading |
| **ESP32 COM** | ESP32-S3 (UART3) | 50Hz | Low (1) | 2KB | Wireless tuning bridge & state broadcast |
| **Monitor** | System Safety | 10Hz | Low (2) | 1KB | System status monitoring |
| **Debug** | Diagnostics | 1Hz | Lowest (1) | 1KB | Debug output |

### Task Communication

```
┌────────────────┐     task notification
│  TIM2 ISR      │──────────────────┐
│  (1kHz, HW)    │  vTaskNotify    │
│  Register-level│  GiveFromISR    │
│  SPI read      │                  │
└────────────────┘                  ▼
                              ┌──────────┐
                              │ IMU      │
                              │ (process)│──queue───┐
                              └──────────┘          │
                                                    ▼
┌─────────┐                              ┌──────────┐
│ Remote  │──────────queue──────────────►│ Balance  │──┐
│ (100Hz) │                              │ (500Hz)  │  │
└─────────┘                              └──────────┘  │
                                                      │
                                    ┌─────────────────┘
                                    ▼
                             ┌──────────────┐
                        ┌───►│ EL05 Motor   │
                        │    │ (100Hz, CAN) │
                        │    └──────────────┘
                        │
                        │    ┌──────────────┐
                        └───►│ M0601C Motor │
                             │ (100Hz, UART)│
                             └──────────────┘
```

### Resource Protection

| Resource | Mutex | Protected Tasks |
|----------|-------|-----------------|
| CAN Bus | `mutex_CAN` | Motor, LQR, Balance |
| SPI3 (NRF24L01) | `mutex_SPI3` | Remote |
| UART1 / RS485 (M0601C) | `mutex_RS485` | Motor |
| UART3 (ESP32) | `mutex_UART_ESP32` | ESP32_COM |

> **Note**: SPI1 (IMU) is accessed only from TIM2 ISR context — no mutex needed. Register-level SPI with loop timeout avoids HAL SysTick dependency.

---

## LQR Control Framework

The system implements a **Linear Quadratic Regulator (LQR)** for wheel-legged robot balancing based on a reduced-order inverted-pendulum-on-wheels model.

### State Space Model

**Reduced state vector (4-dim):**
```
x = [body_angle, body_rate, wheel_position, wheel_velocity]^T
```

**Control input (2-dim):**
```
u = [joint_torque, wheel_torque]^T
```

**Control law:**
```
u = -K * x + Ki * integral(body_angle_error)
```

### LQR Weights

| Weight | Parameter | Default | Effect |
|--------|-----------|---------|--------|
| Q₁₁ | Body angle | 100 | Stiffness of balance |
| Q₂₂ | Body rate | 10 | Damping of body motion |
| Q₃₃ | Wheel position | 1 | Position regulation |
| Q₄₄ | Wheel velocity | 1 | Speed damping |
| R₁₁ | Joint torque effort | 0.1 | Joint energy cost |
| R₂₂ | Wheel torque effort | 0.5 | Wheel energy cost |
| Kᵢ | Integral gain | 0.5 | Steady-state error removal |

### Gain Scheduling

| Mode | Description | Application |
|------|-------------|-------------|
| Standing | High stiffness, aggressive balance | Upright balancing |
| Driving | Reduced stiffness, allows forward motion | Locomotion |
| Sitting | Zero output, motors idle | Resting/startup |
| Emergency Stop | Immediate disable with braking | Safety |

### Key Files

- `Core/Inc/lqr_control.h` - LQR controller API
- `Core/Src/lqr_control.c` - LQR implementation (Riccati solver, state feedback)
- `Core/Inc/robot_model.h` - Robot kinematics and dynamics model
- `Core/Src/robot_model.c` - Model implementation (linearized dynamics)

---

## ESP32-S3 Wireless Tuning System

The dual-MCU architecture enables wireless real-time parameter tuning via an ESP32-S3 coprocessor.

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    User's Phone/PC                       │
│  ┌──────────────────────────────────────────────────┐   │
│  │  Web Browser (Alpine.js + Three.js)              │   │
│  │  ┌──────────┐  ┌──────────┐  ┌───────────────┐  │   │
│  │  │ Parameter│  │  3D Robot│  │  Telemetry    │  │   │
│  │  │ Panel    │  │  Viewer  │  │  Dashboard    │  │   │
│  │  └────┬─────┘  └────┬─────┘  └──────┬────────┘  │   │
│  │       └──────────────┼───────────────┘           │   │
│  └──────────────────────┼───────────────────────────┘   │
│                         │ WebSocket                     │
│                         ▼                               │
│              ┌─────────────────────┐                    │
│              │   ESP32-S3 (AP)     │                    │
│              │  WiFi: WheelRobot-  │                    │
│              │  Tuning             │                    │
│              │  HTTP + WebSocket   │                    │
│              └──────────┬──────────┘                    │
│                         │ UART3 @ 921600                │
│                         ▼                               │
│              ┌─────────────────────┐                    │
│              │   STM32F407         │                    │
│              │  LQR Control        │                    │
│              │  Motor Drivers      │                    │
│              │  IMU Processing     │                    │
│              └─────────────────────┘                    │
└─────────────────────────────────────────────────────────┘
```

### Communication Protocol

UART3 packet format (binary):

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | Start byte 1 (0xAA) |
| 1 | 1 | Start byte 2 (0xBB) |
| 2 | 1 | Packet type |
| 3 | 1 | Payload length |
| 4 | N | Payload data |
| 4+N | 2 | CRC-16 (CCITT) |

**Command types:**
- `0x01` - Set LQR tuning parameters
- `0x02` - Request robot state
- `0x03` - Set control mode
- `0x04` - Enable/Disable motors
- `0x06` - Heartbeat
- `0x20` - System reset

### Key Files

- `Core/Inc/esp32_com.h` - ESP32 communication protocol API
- `Core/Src/esp32_com.c` - Protocol implementation (packet encoding/decoding, CRC)
- `esp32_app/` - ESP32-S3 firmware project (ESP-IDF)

### ESP32-S3 Firmware Build

```bash
cd esp32_app/
idf.py set-target esp32s3
idf.py menuconfig    # Configure WiFi, SPIFFS
idf.py build
idf.py -p PORT flash
```

### Web Frontend

The wireless tuning interface is a single-page application built with:

| Technology | Purpose |
|------------|---------|
| **Alpine.js** | Reactive UI framework |
| **Three.js** | 3D robot visualization with OrbitControls |
| **WebSocket** | Real-time bidirectional data exchange |

**Features:**
- 3D robot model visualization with real-time state updates
- LQR parameter sliders (Q and R matrix weights)
- Control mode selection (Standing, Driving, Sitting, Calibration)
- Real-time telemetry display (IMU, motor status, control output)
- Connection status monitoring with auto-reconnect
- Responsive design for desktop and mobile

**Access:**
1. Connect to WiFi SSID: `WheelRobot-Tuning` (password: `12345678`)
2. Open browser to `http://192.168.4.1`
3. 3D viewport and tuning controls load automatically

### Web Frontend Files

- `esp32_app/data/index.html` - Complete tuning application (Alpine.js + Three.js)
- Served from ESP32-S3 SPIFFS partition

---

## Motor Configuration

### EL05 Joint Motors (CAN Bus)

| Motor ID | Location | CAN ID | Type |
|----------|----------|--------|------|
| M1 | Left Hip | 1 | EL05 |
| M2 | Left Knee | 2 | EL05 |
| M3 | Right Hip | 3 | EL05 |
| M4 | Right Knee | 4 | EL05 |

### M0601C Hub Motors (RS485 Bus)

| Motor ID | Location | RS485 ID | Type |
|----------|----------|----------|------|
| M5 | Left Wheel | 1 | M0601C |
| M6 | Right Wheel | 2 | M0601C |

---

### Performance Metrics

- **CPU Utilization**: ~20% (highly efficient)
- **IMU Read**: 1kHz hardware-timed by TIM2 ISR (deterministic, ±0 µs jitter)
- **IMU Processing**: 1kHz (software filtering in Task_IMU)
- **Balance Control**: 500Hz (real-time response)
- **Motor Control**: 100Hz (precise control)
- **FreeRTOS Heap**: 32KB
- **Total Stack**: ~14KB for all tasks

### Monitoring Variables

Debug counters available in Keil Watch window:

```c
g_imu_update_count           // Should increase by 1000 per second
g_remote_update_count        // Should increase by 100 per second
g_el05_motor_update_count    // Should increase by 100 per second
g_m0601c_motor_update_count  // Should increase by 100 per second
g_can_tx_count               // CAN TX message count
g_can_rx_count               // CAN RX message count
g_system_status              // 0 = all systems normal
```

For detailed FreeRTOS configuration, see [FREERTOS_CONFIG.md](FREERTOS_CONFIG.md).

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
│       ├── can.c                      # CAN initialization
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
### Key Files Added in v2.0 (LQR + Wireless Tuning):

| File | Description |
|------|-------------|
| `Core/Inc/m0601c_motor.h` / `Core/Src/m0601c_motor.c` | M0601C RS485 motor driver with CRC-8 |
| `Core/Inc/lqr_control.h` / `Core/Src/lqr_control.c` | LQR control algorithm framework |
| `Core/Inc/robot_model.h` / `Core/Src/robot_model.c` | Robot kinematics and dynamics model |
| `Core/Inc/esp32_com.h` / `Core/Src/esp32_com.c` | ESP32 UART communication protocol |
| `esp32_app/main/main.c` | ESP32-S3 firmware (WiFi AP + WebSocket) |
| `esp32_app/data/index.html` | Alpine.js + Three.js web tuning UI |

```
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

**M0601C Motor (RS485):**
```
M0601C Motor        STM32F407 (USART1 + RS485)
───────────────────────────────────────────
RS485_A      ───►   TX (PB6)
RS485_B      ───►   RX (PB7)
DIR                  PE0 (DE/RE control)
GND          ───►   GND

Note: USART1 configured as RS485 half-duplex at 115200 baud
      PE0 = HIGH for transmit, LOW for receive
```

**ESP32-S3 (Wireless Tuning Module):**
```
ESP32-S3            STM32F407 (USART3)
───────────────────────────────────────────
TX (GPIO43)  ───►   RX (PB11)
RX (GPIO44)  ───►   TX (PB10)
GND          ───►   GND

Note: 921600 baud, 8N1, full-duplex
      ESP32-S3 acts as WiFi AP: SSID="WheelRobot-Tuning"
      Web interface: http://192.168.4.1
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
| `EL05_WriteParam(motor, addr, value)` | Write float parameter (bytes 4-7) |
| `EL05_WriteParamU8(motor, addr, value)` | Write uint8 parameter (byte 4, for mode sets) |
| `EL05_ReadParam(motor, addr)` | Read parameter |
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
| t_ff | -6 ~ 6 | N·m |

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
| Motor not moving | Wrong CAN ID | Default is 0x7F (127) |
| Motor not moving | Parameter value offset | Write float params at data[4-7], uint8 mode at data[4] |
| Motor oscillating | High gains | Reduce Kp and Kd values |
| CAN communication failed | Baud rate mismatch | Verify 1Mbps setting |
| CAN bus errors | Wiring/termination | Check `g_can_esr` (0 = OK) |
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

- **实时操作系统**：FreeRTOS V10.6.2，7个优先级任务
- **中断驱动IMU**：ICM-42688-P由TIM2硬件中断读取（1kHz，确定性时序）
- **多电机支持**：EL05关节电机（CAN扩展帧）、M0601C轮毂电机（UART）
- **IMU传感器**：ICM-42688-P六轴IMU，带卡尔曼滤波（SPI1）
- **无线控制**：NRF24L01+遥控器，支持对码协议（SPI3）
- **多种控制模式**：MIT模式、位置控制、速度控制、电流控制
- **实时通信**：CAN 2.0 @ 1Mbps，UART @ 115200bps（DMA模式）
- **任务同步**：消息队列、互斥量、信号量
- **STM32 HAL框架**：基于STM32CubeMX生成的代码
- **模块化架构**：独立驱动模块，易于集成

---

## 硬件要求

### 主控制器

| 组件 | 规格 |
|------|------|
| MCU | STM32F407IGHx (UFBGA176封装) |
| 内核 | ARM Cortex-M4 @ 168 MHz，带FPU |
| CAN接口 | CAN1 (PD0: CAN_RX, PD1: CAN_TX) |
| UART接口 | USART1 (PB7: RX, PA9: TX)，带DMA |
| SPI接口 | SPI1 (PA5: SCK, PA6: MISO, PA7: MOSI, PA4: CS) 用于IMU<br>SPI3 (PC10: SCK, PC11: MISO, PC12: MOSI) 用于NRF24L01 |
| 调试接口 | SWD (PA13: SWDIO, PA14: SWCLK) |
| 外部晶振 | 8 MHz HSE |

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
| 通信方式 | CAN 2.0扩展帧 @ 1Mbps |
| 控制模式 | MIT、位置(PP/CSP)、速度、电流 |
| 应用场景 | 轮足机器人关节电机 |

#### M0601C 电机（UART）- 轮毂电机

| 参数 | 数值 |
|------|------|
| 通信方式 | UART @ 115200 bps, 8N1 |
| 帧长度 | 10字节（含CRC-8/MAXIM校验） |
| 控制模式 | 电流环、速度环、位置环 |
| ID范围 | 1-4 |
| 应用场景 | 轮足机器人轮毂电机 |

#### ICM-42688-P 六轴IMU传感器

| 参数 | 数值 |
|------|------|
| 加速度计范围 | ±16g（可配置：±2g, ±4g, ±8g, ±16g） |
| 陀螺仪范围 | ±2000dps（可配置） |
| 输出数据速率 | 1kHz（可配置） |
| 通信方式 | SPI接口 (SPI1) |
| 滤波算法 | 卡尔曼滤波（默认）、移动平均、低通滤波 |
| 温度传感器 | 内置 |

**硬件连接：**
```
ICM-42688-P模块      STM32F407
──────────────────────────────
VCC         ───►   3.3V
GND         ───►   GND
CS          ───►   PA4 (GPIO输出，软件控制)
SCLK        ───►   PA5 (SPI1_SCK)
MISO        ───►   PA6 (SPI1_MISO)
MOSI        ───►   PA7 (SPI1_MOSI)
INT1        ───►   可选（数据就绪中断）
```

**使用示例（中断驱动，1kHz）：**
```c
#include "icm42688.h"
#include "freertos_tasks.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    // 初始化ICM42688（在Task_IMU上下文中调用）
    ICM42688_Init();

    // IMU原始数据由TIM2硬件中断以1kHz读取，
    // 使用寄存器级SPI操作（不依赖HAL/SysTick）。
    // IMU_ISR_Handler() 写入双缓冲区并通过任务通知唤醒Task_IMU。

    while (1) {
        // 处理后的数据通过队列（IMU_Data_t）或监控变量获取：
        float accel_x = g_imu_accel_x_filtered;  // g
        float gyro_x = g_imu_gyro_x_filtered;    // deg/s
        float temp = g_imu_temperature_c;        // °C

        HAL_Delay(10);
    }
}
```

---

## FreeRTOS任务架构

本项目使用**FreeRTOS V10.6.2**，采用基于任务的架构实现实时控制。

### 任务概览

| 任务 | 硬件 | 频率 | 优先级 | 栈大小 | 描述 |
|------|------|------|--------|--------|------|
| **IMU** | ICM-42688-P | 1kHz | 中 | 1KB | IMU数据处理（原始读取由TIM2 ISR完成） |
| **Remote** | NRF24L01+ | 100Hz | 高 | 2KB | 遥控器数据读取 |
| **EL05 Motor** | EL05 (CAN) | 100Hz | 中高 | 2KB | 关节电机控制（CAN） |
| **M0601C Motor** | M0601C (UART) | 100Hz | 中高 | 2KB | 轮毂电机控制（UART） |
| **CAN** | CAN总线 | 500Hz | 高 | 2KB | CAN通信管理 |
| **Balance** | 控制算法 | 500Hz | 高 | 4KB | 平衡控制算法 |
| **Monitor** | 系统安全 | 10Hz | 低 | 1KB | 系统状态监控 |
| **Debug** | 诊断输出 | 1Hz | 最低 | 1KB | 调试输出 |

### 任务通信

```
┌────────────────┐     任务通知
│  TIM2 ISR      │──────────────────┐
│  (1kHz, 硬件)  │  vTaskNotify    │
│  寄存器级SPI   │  GiveFromISR    │
│  读取          │                  │
└────────────────┘                  ▼
                              ┌──────────┐
                              │ IMU      │
                              │ (数据处理) │──队列───┐
                              └──────────┘          │
                                                    ▼
┌─────────┐                              ┌──────────┐
│ Remote  │──────────队列───────────────►│ Balance  │──┐
│ (100Hz) │                              │ (500Hz)  │  │
└─────────┘                              └──────────┘  │
                                                      │
                                    ┌─────────────────┘
                                    ▼
                             ┌──────────────┐
                        ┌───►│ EL05 Motor   │
                        │    │ (100Hz, CAN) │
                        │    └──────────────┘
                        │
                        │    ┌──────────────┐
                        └───►│ M0601C Motor │
                             │ (100Hz, UART)│
                             └──────────────┘
```

### 资源保护

| 资源 | 互斥量 | 保护任务 |
|------|--------|----------|
| CAN总线 | `mutex_CAN` | EL05_Motor, CAN |
| SPI3 (NRF24L01) | `mutex_SPI3` | Remote |
| UART1 (M0601C) | `mutex_UART1` | M0601C_Motor |

> **注意**：SPI1（IMU）仅在TIM2 ISR上下文中访问，无需互斥量保护。寄存器级SPI配合循环超时，不依赖HAL SysTick。

### 性能指标

- **CPU利用率**：~20%（高效）
- **IMU读取**：1kHz由TIM2硬件定时（确定性，±0 µs抖动）
- **IMU处理**：1kHz（Task_IMU中软件滤波）
- **平衡控制**：500Hz（实时响应）
- **电机控制**：100Hz（精确控制）
- **FreeRTOS堆**：32KB
- **总栈大小**：~14KB（所有任务）

### 监控变量

Keil Watch窗口可用的调试计数器：

```c
g_imu_update_count           // 应每秒增加1000
g_remote_update_count        // 应每秒增加100
g_el05_motor_update_count    // 应每秒增加100
g_m0601c_motor_update_count  // 应每秒增加100
g_can_tx_count               // CAN发送计数
g_can_rx_count               // CAN接收计数
g_system_status              // 0 = 所有系统正常
```

详细FreeRTOS配置请参见 [FREERTOS_CONFIG.md](FREERTOS_CONFIG.md)。

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
│       ├── can.c                      # CAN初始化
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
| t_ff | -6 ~ 6 | N·m |

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
| 电机不转 | CAN ID错误 | 默认ID为0x7F (127) |
| 电机不转 | 参数写入格式错误 | float值放data[4-7], 模式值放data[4] |
| 电机抖动 | 增益过高 | 降低Kp和Kd值 |
| CAN通信失败 | 波特率不匹配 | 确认1Mbps设置 |
| CAN总线错误 | 接线/终端电阻 | 检查`g_can_esr` (0=正常) |
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
