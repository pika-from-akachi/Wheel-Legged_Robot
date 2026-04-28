/**
  ******************************************************************************
  * @file    freertos_tasks.h
  * @brief   FreeRTOS task configuration for wheel-legged robot
  * @note    Task priority and stack size definitions
  ******************************************************************************
  */

#ifndef __FREERTOS_TASKS_H__
#define __FREERTOS_TASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "icm42688.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *                          TASK PRIORITY DEFINITIONS
 * ============================================================================
 * Priority levels: 0 (lowest) to 7 (highest)
 * Higher priority tasks can preempt lower priority tasks
 *
 * NOTE: IMU raw data read is handled by TIM2 ISR (HW-timed 1 kHz).
 *       Task_IMU now only does scaling/filtering, so its priority
 *       is lowered from 6 to 3.
 * ============================================================================ */

#define TASK_PRIORITY_IDLE              0   /* Idle task (lowest) */
#define TASK_PRIORITY_DEBUG             1   /* Debug output task */
#define TASK_PRIORITY_MONITOR           2   /* Status monitoring task */
#define TASK_PRIORITY_IMU_PROCESS       3   /* IMU data processing (ISR does the read) */
#define TASK_PRIORITY_REMOTE_CONTROL    3   /* Remote control task */
#define TASK_PRIORITY_MOTOR_CONTROL     4   /* Motor control task */
#define TASK_PRIORITY_BALANCE_CONTROL   5   /* Balance control task */

/* ============================================================================
 *                          TASK STACK SIZE DEFINITIONS
 * ============================================================================
 * Stack sizes in words (4 bytes per word)
 * Adjust based on task complexity and local variables
 * ============================================================================ */

#define TASK_STACK_SIZE_IMU             256     /* IMU processing: 1KB (ISR does the SPI read) */
#define TASK_STACK_SIZE_BALANCE         1024    /* Balance control: 4KB */
#define TASK_STACK_SIZE_MOTOR           512     /* Motor control: 2KB */
#define TASK_STACK_SIZE_REMOTE          512     /* Remote control: 2KB */
#define TASK_STACK_SIZE_MONITOR         256     /* Monitor task: 1KB */
#define TASK_STACK_SIZE_DEBUG           256     /* Debug task: 1KB */

/* ============================================================================
 *                          TASK FREQUENCY DEFINITIONS
 * ============================================================================
 * Task execution frequencies in Hz
 * ============================================================================ */

#define TASK_FREQ_IMU                   1000    /* IMU: 1kHz (timer-triggered ISR) */
#define TASK_FREQ_BALANCE               500     /* Balance: 500Hz */
#define TASK_FREQ_MOTOR                 100     /* Motor: 100Hz */
#define TASK_FREQ_REMOTE                100     /* Remote: 100Hz */
#define TASK_FREQ_MONITOR               10      /* Monitor: 10Hz */
#define TASK_FREQ_DEBUG                 1       /* Debug: 1Hz */

/* ============================================================================
 *                          QUEUE SIZE DEFINITIONS
 * ============================================================================
 * Queue sizes for inter-task communication
 * ============================================================================ */

#define QUEUE_SIZE_IMU_DATA             10      /* IMU data queue */
#define QUEUE_SIZE_REMOTE_DATA          5       /* Remote control data queue */
#define QUEUE_SIZE_MOTOR_CMD            10      /* Motor command queue */

/* ============================================================================
 *                          DATA STRUCTURES
 * ============================================================================ */

/**
  * @brief IMU sensor data structure
  */
typedef struct {
    float accel_x_g;        /* Acceleration X (g) */
    float accel_y_g;        /* Acceleration Y (g) */
    float accel_z_g;        /* Acceleration Z (g) */
    float gyro_x_dps;       /* Angular rate X (deg/s) */
    float gyro_y_dps;       /* Angular rate Y (deg/s) */
    float gyro_z_dps;       /* Angular rate Z (deg/s) */
    float temperature_c;    /* Temperature (°C) */
    uint32_t timestamp;     /* Timestamp (ms) */
} IMU_Data_t;

/**
  * @brief Remote control data structure
  */
typedef struct {
    int16_t right_x;        /* Right joystick X (-128~127) */
    int16_t right_y;        /* Right joystick Y (-128~127) */
    int16_t left_x;         /* Left joystick X (-128~127) */
    int16_t left_y;         /* Left joystick Y (-128~127) */
    uint8_t buttons;        /* Button state */
    uint8_t online;         /* Online status */
    uint32_t timestamp;     /* Timestamp (ms) */
} RemoteData_t;

/**
  * @brief Motor command structure
  */
typedef struct {
    uint8_t motor_id;       /* Motor ID */
    float position;         /* Target position (rad) */
    float velocity;         /* Target velocity (rad/s) */
    float torque;           /* Target torque (N·m) */
    uint8_t mode;           /* Control mode */
    uint32_t timestamp;     /* Timestamp (ms) */
} MotorCmd_t;

/* ============================================================================
 *                          EXTERNAL VARIABLES
 * ============================================================================ */

/* Task handles */
extern osThreadId_t taskHandle_IMU;
extern osThreadId_t taskHandle_Balance;
extern osThreadId_t taskHandle_Motor;
extern osThreadId_t taskHandle_Remote;
extern osThreadId_t taskHandle_Monitor;
extern osThreadId_t taskHandle_Debug;

/* Queue handles */
extern osMessageQueueId_t queue_IMUData;
extern osMessageQueueId_t queue_RemoteData;
extern osMessageQueueId_t queue_MotorCmd;

/* Mutex handles */
extern osMutexId_t mutex_CAN;
extern osMutexId_t mutex_SPI1;
extern osMutexId_t mutex_SPI3;

/* Semaphore handles */
extern osSemaphoreId_t sem_IMU_Ready;
extern osSemaphoreId_t sem_Remote_Ready;

/* ============================================================================
 *                          FUNCTION PROTOTYPES
 * ============================================================================ */

/* Task creation functions */
void FREERTOS_CreateTasks(void);

/* Individual task functions */
void Task_IMU(void *argument);
void Task_Balance(void *argument);
void Task_Motor(void *argument);
void Task_Remote(void *argument);
void Task_Monitor(void *argument);
void Task_Debug(void *argument);

/* IMU ISR and timer functions */
void IMU_ISR_Handler(void);
void IMU_StartTimerInterrupt(void);
ICM42688_RawData_t IMU_GetLatestRawData(void);

/* Utility functions */
void FREERTOS_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __FREERTOS_TASKS_H__ */
