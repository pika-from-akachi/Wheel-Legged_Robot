# FreeRTOS Configuration Guide | FreeRTOS配置指南

[![Platform](https://img.shields.io/badge/Platform-STM32F407-blue.svg)](https://www.st.com)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS_V10.6.2-green.svg)](https://www.freertos.org)

**English** | [中文](#中文文档)

---

## Overview

This guide explains how to configure FreeRTOS for the wheel-legged robot project using STM32CubeMX.

## Step 1: Enable FreeRTOS in STM32CubeMX

### 1.1 Open the Project
1. Open `WheelRobot.ioc` in STM32CubeMX
2. Go to **Middleware** → **FREERTOS**

### 1.2 Configure FreeRTOS
- **Interface**: Select `CMSIS_V2`
- **Configuration** → **Tasks and Queues**:
  - Delete default tasks (we'll create our own)
  - Set `TOTAL_HEAP_SIZE` to `32768` (32KB)

### 1.3 Configure Heap and Stack
- **Heap Size**: `0x400` (1KB, default)
- **Stack Size**: `0x400` (1KB, default)
- **Total Heap Size**: `32768` bytes (32KB for FreeRTOS)

## Step 2: Configure System Settings

### 2.1 Time Base Source
- Go to **System Core** → **SYS**
- **Timebase Source**: Select `TIM1` (not SysTick, as FreeRTOS uses SysTick)

### 2.2 NVIC Settings
- Go to **NVIC** configuration
- Enable **Pendable request for system service**
- Set **Priority Group** to `4 bits for pre-emption priority, 0 bits for subpriority`

## Step 3: Generate Code

1. Click **Project** → **Generate Code**
2. STM32CubeMX will generate:
   - `Core/Src/freertos.c` (FreeRTOS configuration)
   - `Core/Inc/FreeRTOSConfig.h` (FreeRTOS header)
   - Update `main.c` with FreeRTOS initialization

## Step 4: Modify main.c

Replace the main function with FreeRTOS initialization:

```c
/* USER CODE BEGIN Includes */
#include "freertos_tasks.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
// Initialize peripherals
MX_GPIO_Init();
MX_CAN1_Init();
MX_SPI3_Init();
MX_SPI1_Init();

// Initialize FreeRTOS
FREERTOS_Init();

// FreeRTOS kernel starts, program never reaches here
/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  // This code will never execute
  // FreeRTOS tasks run independently
}
/* USER CODE END 3 */
```

## Step 5: Add FreeRTOS Files to Project

### 5.1 Keil MDK-ARM
1. Open `MDK-ARM/WheelRobot.uvprojx`
2. Add `Core/Src/freertos_tasks.c` to project
3. Add `Core/Inc` to include paths

### 5.2 Build Settings
- **C/C++** → **Define**: Add `USE_FREERTOS`
- **Include Paths**: Add `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2`

## Task Architecture

### Task Priority Levels

| Task | Priority | Frequency | Stack Size | Description |
|------|----------|-----------|------------|-------------|
| IMU | 6 (Highest) | 1kHz | 2KB | IMU data acquisition with Kalman filter |
| Balance | 5 | 500Hz | 4KB | Balance control algorithm |
| Motor | 4 | 100Hz | 2KB | Motor control via CAN |
| Remote | 3 | 100Hz | 2KB | Remote control data reading |
| Monitor | 2 | 10Hz | 1KB | System status monitoring |
| Debug | 1 (Lowest) | 1Hz | 1KB | Debug output |

### Task Communication

```
┌─────────┐
│ IMU     │───queue───┐
│ (1kHz)  │            │
└─────────┘            ▼
                 ┌──────────┐
                 │ Balance  │───queue───┐
                 │ (500Hz)  │            │
                 └──────────┘            ▼
                                    ┌─────────┐
                                    │ Motor   │
                                    │ (100Hz) │
                                    └─────────┘

┌──────────┐
│ Remote   │───queue───┐
│ (100Hz)  │            │
└──────────┘            ▼
                 ┌──────────┐
                 │ Balance  │
                 │ (500Hz)  │
                 └──────────┘
```

## Resource Protection

### Mutex Usage

```c
/* Protect CAN bus access */
osMutexAcquire(mutex_CAN, osWaitForever);
EL05_MitControl(&motor, &cmd);
osMutexRelease(mutex_CAN);

/* Protect SPI1 access (IMU) */
osMutexAcquire(mutex_SPI1, osWaitForever);
ICM42688_Update();
osMutexRelease(mutex_SPI1);

/* Protect SPI3 access (NRF24L01) */
osMutexAcquire(mutex_SPI3, osWaitForever);
NRF24L01_RX_ReadData();
osMutexRelease(mutex_SPI3);
```

## Performance Optimization

### CPU Utilization

- **IMU Task**: ~5% CPU (1kHz, Kalman filter)
- **Balance Task**: ~10% CPU (500Hz, control algorithm)
- **Motor Task**: ~3% CPU (100Hz, CAN communication)
- **Remote Task**: ~2% CPU (100Hz, SPI communication)
- **Monitor Task**: ~0.1% CPU (10Hz, status check)
- **Debug Task**: ~0.01% CPU (1Hz, UART output)
- **Total**: ~20% CPU utilization

### Memory Usage

- **FreeRTOS Heap**: 32KB
- **Task Stacks**: 12KB total
- **Queues**: ~500 bytes
- **Available RAM**: ~180KB (STM32F407 has 192KB)

## Debugging Tips

### 1. Task Stack Overflow
- Enable `configCHECK_FOR_STACK_OVERFLOW` in `FreeRTOSConfig.h`
- Increase stack size if overflow detected

### 2. Priority Inversion
- Use mutexes instead of binary semaphores
- Enable priority inheritance

### 3. Watchdog
- Add watchdog refresh in Monitor task
- Detect task starvation

### 4. Performance Analysis
- Use `osThreadGetStackSpace()` to check stack usage
- Use `osKernelGetTickCount()` for timing analysis

## Migration from Bare Metal

### Before (Bare Metal)
```c
while (1) {
    NRF24L01_RX_ReadData();    // Blocking
    ICM42688_Update();          // Blocking
    HAL_Delay(10);              // Wastes CPU time
}
```

### After (FreeRTOS)
```c
void Task_IMU(void *argument) {
    for (;;) {
        ICM42688_Update();      // Non-blocking
        osDelay(1);              // Yields CPU to other tasks
    }
}

void Task_Remote(void *argument) {
    for (;;) {
        NRF24L01_RX_ReadData(); // Non-blocking
        osDelay(10);             // Yields CPU to other tasks
    }
}
```

## Benefits

1. **Real-time Performance**: IMU runs at 1kHz without blocking
2. **Modular Design**: Each task is independent
3. **Safety**: Monitor task can detect and handle failures
4. **Scalability**: Easy to add new tasks
5. **CPU Efficiency**: No wasted time in HAL_Delay()

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Hard fault | Stack overflow | Increase task stack size |
| Task not running | Priority too low | Adjust task priority |
| Queue full | Consumer too slow | Increase queue size or optimize consumer |
| Mutex deadlock | Incorrect mutex usage | Check mutex acquire/release pairs |

---

<br>

---

# 中文文档

[English](#overview) | **中文**

---

## 概述

本指南说明如何使用STM32CubeMX为轮足机器人项目配置FreeRTOS。

## 步骤1：在STM32CubeMX中启用FreeRTOS

### 1.1 打开项目
1. 在STM32CubeMX中打开 `WheelRobot.ioc`
2. 进入 **Middleware** → **FREERTOS**

### 1.2 配置FreeRTOS
- **Interface**: 选择 `CMSIS_V2`
- **Configuration** → **Tasks and Queues**:
  - 删除默认任务（我们将创建自己的任务）
  - 设置 `TOTAL_HEAP_SIZE` 为 `32768`（32KB）

### 1.3 配置堆和栈
- **Heap Size**: `0x400`（1KB，默认）
- **Stack Size**: `0x400`（1KB，默认）
- **Total Heap Size**: `32768` 字节（FreeRTOS使用32KB）

## 步骤2：配置系统设置

### 2.1 时基源
- 进入 **System Core** → **SYS**
- **Timebase Source**: 选择 `TIM1`（不是SysTick，因为FreeRTOS使用SysTick）

### 2.2 NVIC设置
- 进入 **NVIC** 配置
- 启用 **Pendable request for system service**
- 设置 **Priority Group** 为 `4 bits for pre-emption priority, 0 bits for subpriority`

## 步骤3：生成代码

1. 点击 **Project** → **Generate Code**
2. STM32CubeMX将生成：
   - `Core/Src/freertos.c`（FreeRTOS配置）
   - `Core/Inc/FreeRTOSConfig.h`（FreeRTOS头文件）
   - 更新 `main.c` 添加FreeRTOS初始化

## 步骤4：修改main.c

用FreeRTOS初始化替换主函数：

```c
/* USER CODE BEGIN Includes */
#include "freertos_tasks.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
// 初始化外设
MX_GPIO_Init();
MX_CAN1_Init();
MX_SPI3_Init();
MX_SPI1_Init();

// 初始化FreeRTOS
FREERTOS_Init();

// FreeRTOS内核启动，程序永远不会到达这里
/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  // 这段代码永远不会执行
  // FreeRTOS任务独立运行
}
/* USER CODE END 3 */
```

## 步骤5：添加FreeRTOS文件到项目

### 5.1 Keil MDK-ARM
1. 打开 `MDK-ARM/WheelRobot.uvprojx`
2. 将 `Core/Src/freertos_tasks.c` 添加到项目
3. 将 `Core/Inc` 添加到包含路径

### 5.2 编译设置
- **C/C++** → **Define**: 添加 `USE_FREERTOS`
- **Include Paths**: 添加 `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2`

## 任务架构

### 任务优先级

| 任务 | 优先级 | 频率 | 栈大小 | 描述 |
|------|--------|------|--------|------|
| IMU | 6（最高） | 1kHz | 2KB | IMU数据采集（卡尔曼滤波） |
| Balance | 5 | 500Hz | 4KB | 平衡控制算法 |
| Motor | 4 | 100Hz | 2KB | 电机控制（CAN通信） |
| Remote | 3 | 100Hz | 2KB | 遥控器数据读取 |
| Monitor | 2 | 10Hz | 1KB | 系统状态监控 |
| Debug | 1（最低） | 1Hz | 1KB | 调试输出 |

### 任务通信

```
┌─────────┐
│ IMU     │───队列───┐
│ (1kHz)  │            │
└─────────┘            ▼
                 ┌──────────┐
                 │ Balance  │───队列───┐
                 │ (500Hz)  │            │
                 └──────────┘            ▼
                                    ┌─────────┐
                                    │ Motor   │
                                    │ (100Hz) │
                                    └─────────┘

┌──────────┐
│ Remote   │───队列───┐
│ (100Hz)  │            │
└──────────┘            ▼
                 ┌──────────┐
                 │ Balance  │
                 │ (500Hz)  │
                 └──────────┘
```

## 资源保护

### 互斥量使用

```c
/* 保护CAN总线访问 */
osMutexAcquire(mutex_CAN, osWaitForever);
EL05_MitControl(&motor, &cmd);
osMutexRelease(mutex_CAN);

/* 保护SPI1访问（IMU） */
osMutexAcquire(mutex_SPI1, osWaitForever);
ICM42688_Update();
osMutexRelease(mutex_SPI1);

/* 保护SPI3访问（NRF24L01） */
osMutexAcquire(mutex_SPI3, osWaitForever);
NRF24L01_RX_ReadData();
osMutexRelease(mutex_SPI3);
```

## 性能优化

### CPU利用率

- **IMU任务**: ~5% CPU（1kHz，卡尔曼滤波）
- **Balance任务**: ~10% CPU（500Hz，控制算法）
- **Motor任务**: ~3% CPU（100Hz，CAN通信）
- **Remote任务**: ~2% CPU（100Hz，SPI通信）
- **Monitor任务**: ~0.1% CPU（10Hz，状态检查）
- **Debug任务**: ~0.01% CPU（1Hz，UART输出）
- **总计**: ~20% CPU利用率

### 内存使用

- **FreeRTOS堆**: 32KB
- **任务栈**: 总计12KB
- **队列**: ~500字节
- **可用RAM**: ~180KB（STM32F407有192KB）

## 调试技巧

### 1. 任务栈溢出
- 在 `FreeRTOSConfig.h` 中启用 `configCHECK_FOR_STACK_OVERFLOW`
- 如果检测到溢出，增加栈大小

### 2. 优先级反转
- 使用互斥量而不是二值信号量
- 启用优先级继承

### 3. 看门狗
- 在Monitor任务中添加看门狗刷新
- 检测任务饥饿

### 4. 性能分析
- 使用 `osThreadGetStackSpace()` 检查栈使用情况
- 使用 `osKernelGetTickCount()` 进行时序分析

## 从裸机迁移

### 迁移前（裸机）
```c
while (1) {
    NRF24L01_RX_ReadData();    // 阻塞
    ICM42688_Update();          // 阻塞
    HAL_Delay(10);              // 浪费CPU时间
}
```

### 迁移后（FreeRTOS）
```c
void Task_IMU(void *argument) {
    for (;;) {
        ICM42688_Update();      // 非阻塞
        osDelay(1);              // 让出CPU给其他任务
    }
}

void Task_Remote(void *argument) {
    for (;;) {
        NRF24L01_RX_ReadData(); // 非阻塞
        osDelay(10);             // 让出CPU给其他任务
    }
}
```

## 优势

1. **实时性能**: IMU以1kHz运行，不会阻塞
2. **模块化设计**: 每个任务独立
3. **安全性**: Monitor任务可以检测和处理故障
4. **可扩展性**: 易于添加新任务
5. **CPU效率**: HAL_Delay()不浪费时间

## 故障排除

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 硬件错误 | 栈溢出 | 增加任务栈大小 |
| 任务不运行 | 优先级太低 | 调整任务优先级 |
| 队列满 | 消费者太慢 | 增加队列大小或优化消费者 |
| 互斥量死锁 | 互斥量使用不当 | 检查互斥量获取/释放配对 |

---

**Happy Coding! | 祝开发顺利！**
