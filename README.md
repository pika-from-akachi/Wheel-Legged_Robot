# Wheel-Legged Robot Control System | 轮足机器人控制系统

[![Platform](https://img.shields.io/badge/Platform-STM32F407-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407-417.html)
[![Framework](https://img.shields.io/badge/Framework-STM32CubeMX_6.15.0-green.svg)](https://www.st.com/en/development-tools/stm32cubemx.html)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS_V10.3.1-orange.svg)](https://www.freertos.org)
[![Language](https://img.shields.io/badge/Language-C-orange.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**English** | [**中文**](#中文文档)

> 🔄 [English Version](#overview) · [中文版本](#中文文档)

---

## Overview

This project implements the embedded control system for a wheel-legged robot, developed for the **2026 Mingyue Class**. The system is based on the STM32F407IGHx microcontroller and supports multiple motor types through CAN and UART communication interfaces.

### Key Features

- **Real-Time Operating System**: FreeRTOS V10.3.1 with 8 priority-based tasks (6 active, 2 disabled)
- **Interrupt-Driven IMU**: ICM-42688-P read by TIM2 hardware interrupt (1 kHz, deterministic timing)
- **Multi-Motor Support**: EL05 joint motor (CAN extended frame), M0601C wheel motor (RS485)
- **Dual Wheel Drive**: Left wheel (ID=1) and right wheel (ID=2) with differential speed control
- **LQR Balance Controller**: PD+I+Kv+Kpos with empirically tuned gains and speed/position loops
- **Dual MCU Architecture**: STM32F407 (control) + ESP32-S3 (wireless tuning + web remote)
- **Web Remote Control**: Touch joystick drive + light toggle via ESP32 hotspot (192.168.4.1/remote)
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
| RS485 Interface | USART2 (PD5: RX, PD6: TX) with THVD1410DR (PD3: RE#, PD4: DE) |
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
- Auto-retransmit: 8 retries, 2250µs delay

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

#### M0601C Motor (RS485) - Wheel Motor

| Parameter | Value |
|-----------|-------|
| Communication | RS485 (USART2) via THVD1410DR @ 115200 bps, 8N1 |
| Frame Length | 10 bytes (with CRC-8/MAXIM) |
| Control Modes | Current, Speed, Position |
| ID Range | 1-4 |
| Application | Wheel motor for wheel-legged robot |
| Left Wheel | ID=1 (forward: +RPM / reverse: -RPM) |
| Right Wheel | ID=2 (forward: -RPM / reverse: +RPM) |

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

This project uses **FreeRTOS V10.3.1** with a comprehensive task-based architecture for real-time control.

### Task Overview

| Task | Hardware | Frequency | Priority | Stack | Description |
|------|----------|-----------|----------|-------|-------------|
| **IMU** | ICM-42688-P | 1kHz (ISR唤醒) | AboveNormal | 4KB | IMU data processing (raw read by TIM2 ISR) |
| **Remote** | NRF24L01+ | 100Hz | AboveNormal1 | 8KB | Remote controller data reading |
| **EL05 Motor** | EL05 (CAN) + M0601C (RS485) | 10Hz | AboveNormal | 8KB | **Single task**: 4 joint motors + both wheel motors |
| **Balance** | IMU + LQR | 200Hz | AboveNormal2 | 16KB | LQR balance control (IMU→filter→LQR→motors) |
| **Monitor** | System Safety | 10Hz | BelowNormal | 4KB | System status monitoring |
| **Debug** | Diagnostics | 1Hz | Low | 4KB | Debug output |
| ~~M0601C Motor~~ | M0601C (RS485) | (disabled) | - | - | Wheels controlled within EL05 Motor task |
| ~~CAN~~ | CAN Bus | (disabled) | - | - | CAN processing merged into EL05 Motor task |

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
│ (100Hz) │                              │ (200Hz)  │  │
└─────────┘                              └──────────┘  │
                                                      │
                                    ┌─────────────────┘
                                    ▼
                             ┌──────────────┐
                             │ EL05 Motor   │
                             │ (10Hz, CAN + │
                             │  RS485)      │
                             └──────────────┘
```

### Resource Protection

| Resource | Mutex | Protected Tasks |
|----------|-------|-----------------|
| CAN Bus | `mutex_CAN` | EL05 Motor, Balance |
| SPI1 (IMU) | `mutex_SPI1` | IMU (guard configuration) |
| SPI3 (NRF24L01) | `mutex_SPI3` | Remote |
| UART1 / RS485 (M0601C) | `mutex_UART1` | EL05 Motor |
| UART3 (ESP32) | `mutex_UART3` | ESP32_COM (if enabled) |

> **Note**: SPI1 (IMU) is accessed only from TIM2 ISR context — no mutex needed. Register-level SPI with loop timeout avoids HAL SysTick dependency.

---

## LQR Balance Controller | LQR平衡控制器

The balancing controller uses a **PD + Integral + Velocity Loop + Position Loop** architecture on a reduced 4-state inverted-pendulum-on-wheels model. Gains are tuned empirically rather than solved online via Riccati.

> 平衡控制器采用 **PD + 积分 + 速度环 + 位置环** 架构。增益通过实测调谐而非在线 Riccati 求解。

### Control Law | 控制律

```
u_wheel = -(P·θ + D·ω + Kpos·pos + Kv·vel) - Ki·∫θdt
```

Where `θ = body_angle`, `ω = body_rate`, `pos = wheel_position`, `vel = wheel_velocity`.

| Term | Description | 描述 |
|:-----|:------------|:-----|
| P·θ | Body angle stiffness | 体角刚度，倾斜→恢复力 |
| D·ω | Body rate damping | 角速度阻尼，防过冲 |
| Kpos·pos | Position loop — pulls back to origin | 位置环，走远了拉回 |
| Kv·vel | Velocity loop — resists wheel slip | 速度环，轮速→零速锁定 |
| Ki·∫θdt | Integral — eliminates steady drift | 积分项，消除残留偏移 |

### Tuned Gains (Current) | 当前调谐参数

| Gain | Value | Unit | Description |
|:-----|:-----:|:----:|:------------|
| **P** | **+2.0** | Nm/rad | Body angle stiffness / 体角刚度 |
| **D** | **+0.3** | Nm/(rad/s) | Body rate damping / 角速度阻尼 |
| **Kpos** | **+0.1** | Nm/rad | Wheel position loop / 位置环 |
| **Kv** | **+2.7** | Nm/(rad/s) | Wheel velocity loop / 速度环 |
| **Ki** | **3.0** | Nm/(rad·s) | Integral gain (limit ±0.3 Nm) / 积分增益(限幅0.3) |
| g_accel_offset | 3.096 | rad | IMU balance zero (177.4°) / IMU平衡零点 |

> **Note:** Positive gains verified by motor direction test. `u = -(K·x)` → forward tilt (θ>0) produces negative torque → wheel forward → correction.
> **注:** 正增益经由电机方向测试验证。前倾(θ>0)→负扭矩→轮子向前→修正。

### State Estimation | 状态估计

```
body_angle = complementary_filter(gyro_integral, accel_angle)
           = α·(angle + gyro·dt) + (1-α)·atan2(accel_y, accel_z)
           where α = 0.90 (freertos.c) or 0.98 (freertos_tasks.c)

body_rate  = gyro_x (negated for X/Z-flipped IMU)

wheel_vel  = M0601C_Motor.feedback.speed_rpm × 0.10472 (RPM→rad/s)
wheel_pos  = ∫ wheel_vel · dt  (×0.999 decay)
```

### IMU Calibration | IMU校准

1. Z-up orientation verified by 3-pose test (upright, forward-tilt, backward-tilt)
2. `g_accel_offset = atan2(ay, az) at upright ≈ π` (hardcoded to 3.096 rad / 177.4°)
3. Gyro bias set to 0 (complementary filter auto-converges)
4. Motor-task auto-calibration disabled (offset is hardcoded)

### Tuning History | 调参历程

| Step | P | D | Ki | Kv | Kpos | Note |
|:-----|:--:|:--:|:--:|:--:|:--:|:-----|
| Initial | 0.5 | 0.2 | 0 | 0 | 0 | 原始代码，摆幅大 |
| Fix IMU | 0.5 | 0.2 | 0 | 0 | 0 | 修正IMU方向(倒装→正装) |
| Tune P up | 0.5→1.3 | 0.2 | 0 | 0 | 0 | 逐步提P，定点稳定 |
| Add Ki | 1.3 | 0.2 | 0.2→3.0 | 0 | 0 | 积分方向修正(+=→−=) |
| Add Kv | 1.3 | 0.2 | 0.8 | 0→2.7 | 0 | 速度环定点锁定 |
| Add Kpos | 1.8 | 0.3 | 3.0 | 2.7 | 0.1 | 位置环归零 |
| Current | **2.0** | **0.3** | **3.0** | **2.7** | **0.1** | 最终调谐 |

### Key Files | 关键文件

| File | Description |
|:-----|:------------|
| `project/Core/Inc/lqr_control.h` | LQR controller API + default parameters |
| `project/Core/Src/lqr_control.c` | LQR implementation (state feedback + integral) |
| `project/Core/Inc/robot_model.h` | Robot kinematics, state definitions |
| `project/Core/Src/robot_model.c` | Gain matrix K[2][4], linearized dynamics |
| `project/Core/Src/freertos.c` | Task_Balance (200Hz): IMU→filter→LQR→motors |
| `project/Core/Src/icm42688.c` | ICM-42688-P IMU driver (SPI, 1kHz ISR) |

### Safety | 安全

- Integral only active when `|θ| < 0.2 rad` (conditional anti-windup)
- Wheel torque saturated to ±3.0 Nm
- Body angle exceeds ±0.52 rad (±30°) → emergency condition
- Torque deadband: ±0.02 Nm → zero output

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
- `0x05` - Direct motor command
- `0x06` - Heartbeat
- `0x07` - Set target speed (web remote)
- `0x08` - Set target joint positions
- `0x09` - M0601C-specific command
- `0x10` - Remote controller button event
- `0x20` - System reset
- `0x30` - Firmware version query

**Response types:**
- `0x81` - ACK
- `0x82` - State data (45-byte: 24 IMU + 12 M0601C + 8 LQR + 1 mode)
- `0x83` - Tuning data
- `0x84` - Motor feedback
- `0x8F` - Error
- `0xF0` - Debug stream

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

| Motor ID | Location | CAN ID | Index | Type |
|----------|----------|--------|-------|------|
| M1 | Left Hip Front | 1 | 0 | EL05 |
| M2 | Right Hip Front | 2 | 1 | EL05 |
| M3 | Left Hip Rear | 3 | 2 | EL05 (mirror of M1) |
| M4 | Right Hip Rear | 4 | 3 | EL05 (mirror of M2) |

### Action Group 0: Min Leg Height (动作组0: 最小腿高/归零)

Pure homing — all 4 joint motors return to their mechanical zero positions (minimum leg height).

| Motor | CAN ID | Zero Approach | Zero Position |
|-------|--------|---------------|---------------|
| M1 | 1 | CCW small angle | **0 rad** |
| M2 | 2 | CW via 2π | **6.2832 rad** |
| M3 | 3 | CW via 2π | **6.2832 rad** |
| M4 | 4 | CCW small angle | **0 rad** |

```c
// freertos.c — Action Group 0: homing positions
static const float g_action0[4] = {
    0.0f,      /* M1/idx0: CCW zero */
    6.2832f,   /* M2/idx1: CW zero(2π) */
    6.2832f,   /* M3/idx2: CW zero(2π) mirror */
    0.0f,      /* M4/idx3: CCW zero mirror */
};
```

### Action Group 1: Init Max Leg Height (动作组1: 初始化到最大腿高)

Go to zero first (Action Group 0), then rotate to max leg height positions.

| Motor | CAN ID | Target Position |
|-------|--------|-----------------|
| M1 | 1 | **+1.5708 rad (+90°)** |
| M2 | 2 | **4.3633 rad (-110°)** |
| M3 | 3 | **4.7124 rad (-90°)** |
| M4 | 4 | **+1.9199 rad (+110°)** |

Control sequence: `Configure PP mode → Disable(clear fault) → Enable → Apply Action Group 0 (homing) → Apply Action Group 1 (target) → 10Hz hold loop`

```c
// freertos.c — Action Group 0 (homing) + dual wheel drive
static const float g_action0[4] = { /* homing: 0, 6.2832, 6.2832, 0 */ };

void Task_EL05_Motor(void *argument) {
    /* Step 1-2: Configure PP mode & enable all 4 joint motors */
    /* Step 3: Apply g_action0[i] → homing (min leg height) */
    /* Step 4: Switch both wheel motors to speed mode */
    /*         MOTOR_SetSpeed(1, +50)  — left wheel forward */
    /*         MOTOR_SetSpeed(2, -50)  — right wheel forward (opposite) */
    /* Step 5: 10Hz loop — hold joint positions + maintain wheel speeds */
}
```

### M0601C Hub Motors (RS485 Bus)

| Motor | Location | RS485 ID | Type | Direction |
|-------|----------|----------|------|-----------|
| M5 | Left Wheel | 1 | M0601C | +RPM = forward |
| M6 | Right Wheel | 2 | M0601C | -RPM = forward (opposite) |

**Differential Drive:**
- Forward: `MOTOR_SetSpeed(1, +50)` / `MOTOR_SetSpeed(2, -50)`
- Reverse: `MOTOR_SetSpeed(1, -50)` / `MOTOR_SetSpeed(2, +50)`
- Turn left: `MOTOR_SetSpeed(1, 0)` / `MOTOR_SetSpeed(2, -50)`
- Turn right: `MOTOR_SetSpeed(1, +50)` / `MOTOR_SetSpeed(2, 0)`

---

### Performance Metrics

- **CPU Utilization**: ~20% (highly efficient)
- **IMU Read**: 1kHz hardware-timed by TIM2 ISR (deterministic, ±0 µs jitter)
- **IMU Processing**: 1kHz (software filtering in Task_IMU)
- **Balance Control**: 200Hz (real-time response)
- **Motor Control**: 10Hz (joint position hold loop)
- **FreeRTOS Heap**: 32KB (heap_4)
- **Total Stack**: ~45KB for active tasks (11,264 words × 4 bytes)

### Monitoring Variables

Debug counters available in Keil Watch window:

```c
g_imu_update_count            // Should increase by 1000 per second
g_remote_update_count         // Should increase by 100 per second
g_el05_motor_update_count     // Should increase by 10 per second
g_m0601c_motor_update_count   // M0601C motor control counter
g_can_tx_count                // CAN TX message count
g_can_rx_count                // CAN RX message count
g_system_status               // 0 = all systems normal
debug_wheel_cmd_status        // Left wheel (ID=1) cmd: 1=OK, 2=fail
debug_right_wheel_status      // Right wheel (ID=2) cmd: 1=OK, 2=fail
dr0                           // RS485 feedback frame DATA[0] = motor ID
g_can_esr                     // CAN Error Status Register (0 = OK)
g_can_tsr                     // CAN Transmit Status Register
```

For detailed FreeRTOS configuration, see [FREERTOS_CONFIG.md](FREERTOS_CONFIG.md).

---

## Project Structure

```
Wheel-Legged_Robot/
├── project/Core/                       # Main application code
│   ├── Inc/                            # Header files
│   │   ├── main.h
│   │   ├── can.h
│   │   ├── freertos_tasks.h
│   │   ├── lqr_control.h
│   │   ├── robot_model.h
│   │   ├── esp32_com.h
│   │   ├── el05_motor.h
│   │   ├── m0601c_motor.h
│   │   ├── icm42688.h
│   │   ├── nrf24l01_rx.h
│   │   └── gpio.h
│   └── Src/                            # Source files
│       ├── main.c                      # Main entry point
│       ├── freertos.c                  # FreeRTOS task creation
│       ├── freertos_tasks.c            # Task implementations
│       ├── can.c                       # CAN initialization
│       ├── lqr_control.c               # LQR balance controller
│       ├── robot_model.c               # Robot kinematics & dynamics
│       ├── esp32_com.c                 # ESP32 UART protocol
│       ├── el05_motor.c                # EL05 CAN motor driver
│       ├── m0601c_motor.c              # M0601C RS485 motor driver
│       ├── icm42688.c                  # ICM-42688-P IMU driver
│       ├── nrf24l01_rx.c               # NRF24L01+ receiver
│       └── gpio.c                      # GPIO configuration
│
├── project/Drivers/                    # STM32 HAL & CMSIS libraries
├── project/MDK-ARM/                    # Keil MDK project files
### Key Files Added in v2.0 (LQR + Wireless Tuning):

| File | Description |
|------|-------------|
| `project/Core/Inc/lqr_control.h` / `project/Core/Src/lqr_control.c` | LQR control algorithm framework |
| `project/Core/Inc/robot_model.h` / `project/Core/Src/robot_model.c` | Robot kinematics and dynamics model |
| `project/Core/Inc/esp32_com.h` / `project/Core/Src/esp32_com.c` | ESP32 UART communication protocol |
| `project/Core/Inc/m0601c_motor.h` / `project/Core/Src/m0601c_motor.c` | M0601C RS485 motor driver with CRC-8 |
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

**M0601C Motor (RS485 via USART2):**
```
M0601C Motor        STM32F407 (USART2 + THVD1410DR)
───────────────────────────────────────────
RS485_A      ───►   A (PD6, via THVD1410DR)
RS485_B      ───►   B (PD5, via THVD1410DR)
GND          ───►   GND
RE/DE control       PD3 (RE#), PD4 (DE)

Note: USART2 configured as RS485 half-duplex at 115200 baud
      THVD1410DR: PD3=RE# (LOW=receive), PD4=DE (HIGH=transmit)
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
| p_des | -12.57 ~ 12.57 | rad |
| v_des | -50 ~ 50 | rad/s |
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

- [EL05 Motor User Manual](project/Core/Inc/el05_motor.h)
- [NRF24L01+ Wireless Module Driver](project/Core/Inc/nrf24l01_rx.h)
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

- **实时操作系统**：FreeRTOS V10.3.1，8个优先级任务（6个活跃，2个禁用）
- **中断驱动IMU**：ICM-42688-P由TIM2硬件中断读取（1kHz，确定性时序）
- **多电机支持**：EL05关节电机（CAN扩展帧）、M0601C轮毂电机（RS485）
- **双轮差速驱动**：左轮(ID=1)正转前进，右轮(ID=2)反转前进
- **IMU传感器**：ICM-42688-P六轴IMU，带卡尔曼滤波（SPI1）
- **无线控制**：NRF24L01+遥控器，支持对码协议（SPI3）
- **多种控制模式**：MIT模式、位置控制、速度控制、电流控制
- **实时通信**：CAN 2.0 @ 1Mbps，RS485 @ 115200bps，UART @ 921600bps
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
| RS485接口 | USART2 (PD5: RX, PD6: TX) via THVD1410DR (PD3: RE#, PD4: DE) |
| UART接口 | USART1 (PB7: RX, PA9: TX)，带DMA |
| ESP32接口 | USART3 (PB10: TX, PB11: RX) @ 921600 baud |
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
- 自动重传：8次重试，2250µs延时

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

#### M0601C 电机（RS485）- 轮毂电机

| 参数 | 数值 |
|------|------|
| 通信方式 | RS485 (USART2) via THVD1410DR @ 115200 bps, 8N1 |
| 帧长度 | 10字节（含CRC-8/MAXIM校验） |
| 控制模式 | 电流环、速度环、位置环 |
| ID范围 | 1-4 |
| 应用场景 | 轮足机器人轮毂电机 |
| 左轮毂 | ID=1（正转前进，反转后退） |
| 右轮毂 | ID=2（反转前进，正转后退） |

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

本项目使用**FreeRTOS V10.3.1**，采用基于任务的架构实现实时控制。

### 任务概览

| 任务 | 硬件 | 频率 | 优先级 | 栈大小 | 描述 |
|------|------|------|--------|--------|------|
| **IMU** | ICM-42688-P | 1kHz (ISR唤醒) | AboveNormal | 4KB | IMU数据处理（原始读取由TIM2 ISR完成） |
| **Remote** | NRF24L01+ | 100Hz | AboveNormal1 | 8KB | 遥控器数据读取 |
| **EL05 Motor** | EL05 (CAN) + M0601C (RS485) | 10Hz | AboveNormal | 8KB | **单任务**: 4个关节电机 + 左右轮毂电机 |
| **Balance** | IMU + LQR | 200Hz | AboveNormal2 | 16KB | LQR平衡控制 (IMU→滤波→LQR→电机) |
| **Monitor** | 系统安全 | 10Hz | BelowNormal | 4KB | 系统状态监控 |
| **Debug** | 诊断输出 | 1Hz | Low | 4KB | 调试输出 |
| ~~M0601C Motor~~ | M0601C (RS485) | (已禁用) | - | - | 轮毂控制已合并到EL05 Motor任务 |
| ~~CAN~~ | CAN总线 | (已禁用) | - | - | CAN处理已合并到EL05 Motor任务 |

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
│ (100Hz) │                              │ (200Hz)  │  │
└─────────┘                              └──────────┘  │
                                                      │
                                    ┌─────────────────┘
                                    ▼
                             ┌──────────────┐
                             │ EL05 Motor   │
                             │ (10Hz, CAN + │
                             │  RS485)      │
                             └──────────────┘
```

### 资源保护

| 资源 | 互斥量 | 保护任务 |
|------|--------|----------|
| CAN总线 | `mutex_CAN` | EL05 Motor, Balance |
| SPI1 (IMU) | `mutex_SPI1` | IMU (配置保护) |
| SPI3 (NRF24L01) | `mutex_SPI3` | Remote |
| UART1 / RS485 (M0601C) | `mutex_UART1` | EL05 Motor |

> **注意**：SPI1（IMU）仅在TIM2 ISR上下文中访问，无需互斥量保护。寄存器级SPI配合循环超时，不依赖HAL SysTick。

### 性能指标

- **CPU利用率**：~20%（高效）
- **IMU读取**：1kHz由TIM2硬件定时（确定性，±0 µs抖动）
- **IMU处理**：1kHz（Task_IMU中软件滤波）
- **平衡控制**：200Hz（实时响应）
- **电机控制**：10Hz（关节位置保持循环）
- **FreeRTOS堆**：32KB（heap_4）
- **总栈大小**：~45KB（活跃任务，11,264 words × 4字节）

### 监控变量

Keil Watch窗口可用的调试计数器：

```c
g_imu_update_count           // 应每秒增加1000
g_remote_update_count        // 应每秒增加100
g_el05_motor_update_count    // 应每秒增加10
g_can_tx_count               // CAN发送计数
g_can_rx_count               // CAN接收计数
g_system_status              // 0 = 所有系统正常
debug_wheel_cmd_status       // 左轮(ID=1)指令状态: 1=成功, 2=失败
debug_right_wheel_status     // 右轮(ID=2)指令状态: 1=成功, 2=失败
dr0                          // RS485反馈帧DATA[0] = 电机ID
```

详细FreeRTOS配置请参见 [FREERTOS_CONFIG.md](FREERTOS_CONFIG.md)。

---

## 项目结构

```
Wheel-Legged_Robot/
├── project/Core/                       # 主应用代码
│   ├── Inc/                            # 头文件
│   │   ├── main.h
│   │   ├── can.h
│   │   ├── freertos_tasks.h
│   │   ├── lqr_control.h
│   │   ├── robot_model.h
│   │   ├── esp32_com.h
│   │   ├── el05_motor.h
│   │   ├── m0601c_motor.h
│   │   ├── icm42688.h
│   │   ├── nrf24l01_rx.h
│   │   └── gpio.h
│   └── Src/                            # 源文件
│       ├── main.c                      # 主程序入口
│       ├── freertos.c                  # FreeRTOS任务创建
│       ├── freertos_tasks.c            # 任务实现
│       ├── can.c                       # CAN初始化
│       ├── lqr_control.c               # LQR平衡控制
│       ├── robot_model.c               # 机器人运动学与动力学
│       ├── esp32_com.c                 # ESP32 UART协议
│       ├── el05_motor.c                # EL05 CAN电机驱动
│       ├── m0601c_motor.c              # M0601C RS485电机驱动
│       ├── icm42688.c                  # ICM-42688-P IMU驱动
│       ├── nrf24l01_rx.c               # NRF24L01+ 接收器
│       └── gpio.c                      # GPIO配置
│
├── project/Drivers/                    # STM32 HAL及CMSIS库
├── project/MDK-ARM/                    # Keil MDK工程文件
└── WheelRobot.ioc                      # STM32CubeMX配置文件
```

---

## 电机配置

### EL05关节电机（CAN总线）

| 电机编号 | 安装位置 | CAN ID | 数组索引 | 说明 |
|----------|----------|--------|----------|------|
| M1 | 左腿髋前 | 1 | 0 | EL05 |
| M2 | 右腿髋前 | 2 | 1 | EL05 |
| M3 | 左腿髋后 | 3 | 2 | EL05 (M1镜像) |
| M4 | 右腿髋后 | 4 | 3 | EL05 (M2镜像) |

### 动作组0: 最小腿高 (归零位置)

纯归零动作 — 4个关节电机全部回到机械零点位置，即最小腿高。

| 电机 | CAN ID | 归零方式 | 归零位置 |
|------|--------|----------|----------|
| M1 | 1 | 逆时针小角度 | **0 rad** |
| M2 | 2 | 顺时针走2π | **6.2832 rad** |
| M3 | 3 | 顺时针走2π | **6.2832 rad** |
| M4 | 4 | 逆时针小角度 | **0 rad** |

```c
// freertos.c — 动作组0: 归零位置
static const float g_action0[4] = {
    0.0f,      /* M1/idx0: 逆时针归零 */
    6.2832f,   /* M2/idx1: 顺时针2π归零 */
    6.2832f,   /* M3/idx2: 顺时针2π归零 镜像 */
    0.0f,      /* M4/idx3: 逆时针归零 镜像 */
};
```

### 动作组1: 初始化到最大腿高

先执行动作组0归零（最小腿高），再转到最大腿高位置。

| 电机 | CAN ID | 目标位置 |
|------|--------|----------|
| M1 | 1 | **+1.5708 rad (+90°)** |
| M2 | 2 | **4.3633 rad (-110°)** |
| M3 | 3 | **4.7124 rad (-90°)** |
| M4 | 4 | **+1.9199 rad (+110°)** |

控制顺序：`配置PP模式 → Disable清故障 → Enable → 动作组0(归零) → 动作组1(目标) → 10Hz保持循环`

```c
// freertos.c — 动作组0(归零) + 双轮毂驱动
static const float g_action0[4] = { /* 归零: 0, 6.2832, 6.2832, 0 */ };

void Task_EL05_Motor(void *argument) {
    /* Step 1-2: 配置PP模式 & 使能4个关节电机 */
    /* Step 3: g_action0[i] → 最小腿高(归零) */
    /* Step 4: 左右轮毂切速度模式 */
    /*         MOTOR_SetSpeed(1, +50)  — 左轮正转(前进) */
    /*         MOTOR_SetSpeed(2, -50)  — 右轮反转(前进) */
    /* Step 5: 10Hz循环 — 保持关节位置 + 维持轮毂转速 */
}
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

**M0601C电机（RS485 via USART2）：**
```
M0601C电机          STM32F407 (USART2 + THVD1410DR)
────────────────────────────────────────────
RS485_A      ───►   A (PD6, via THVD1410DR)
RS485_B      ───►   B (PD5, via THVD1410DR)
GND          ───►   GND
RE/DE控制           PD3 (RE#), PD4 (DE)

注意：USART2配置为RS485半双工，115200波特率
      THVD1410DR: PD3=RE#(LOW=接收), PD4=DE(HIGH=发送)
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
| p_des | -12.57 ~ 12.57 | rad |
| v_des | -50 ~ 50 | rad/s |
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
| UART/RS485无响应 | 引脚错误 | 检查PD5(RX)/PD6(TX) (USART2 + THVD1410DR) |
| CRC错误 | 信号干扰 | 检查线缆屏蔽 |

---

## 开发工具

- **STM32CubeMX** v6.15.0 - 配置工具
- **Keil MDK-ARM** V5.32+ - ARM编译器
- **IAR EWARM** - 备选工具链
- **ST-Link** - 调试探针

---

## 参考资料

- [EL05电机使用手册](project/Core/Inc/el05_motor.h)
- [NRF24L01+ 无线模块驱动](project/Core/Inc/nrf24l01_rx.h)
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
