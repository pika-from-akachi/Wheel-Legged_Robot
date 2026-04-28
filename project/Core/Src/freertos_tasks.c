/**
  ******************************************************************************
  * @file    freertos_tasks.c
  * @brief   FreeRTOS task implementations for wheel-legged robot
  ******************************************************************************
  */

#include "freertos_tasks.h"
#include "icm42688.h"
#include "nrf24l01_rx.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ============================================================================
 *                          EXTERNAL VARIABLES
 * ============================================================================ */

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern CAN_HandleTypeDef hcan1;

/* Debug variables */
extern volatile float g_imu_accel_x_filtered;
extern volatile float g_imu_accel_y_filtered;
extern volatile float g_imu_accel_z_filtered;
extern volatile float g_imu_gyro_x_filtered;
extern volatile float g_imu_gyro_y_filtered;
extern volatile float g_imu_gyro_z_filtered;
extern volatile float g_imu_temperature_c;

/* ============================================================================
 *                          GLOBAL VARIABLES
 * ============================================================================ */

/* Task handles */
osThreadId_t taskHandle_IMU = NULL;
osThreadId_t taskHandle_Balance = NULL;
osThreadId_t taskHandle_Motor = NULL;
osThreadId_t taskHandle_Remote = NULL;
osThreadId_t taskHandle_Monitor = NULL;
osThreadId_t taskHandle_Debug = NULL;

/* Queue handles */
osMessageQueueId_t queue_IMUData = NULL;
osMessageQueueId_t queue_RemoteData = NULL;
osMessageQueueId_t queue_MotorCmd = NULL;

/* Mutex handles */
osMutexId_t mutex_CAN = NULL;
osMutexId_t mutex_SPI1 = NULL;
osMutexId_t mutex_SPI3 = NULL;

/* Semaphore handles */
osSemaphoreId_t sem_IMU_Ready = NULL;
osSemaphoreId_t sem_Remote_Ready = NULL;

/* Status variables */
volatile uint8_t g_system_status = 0;
volatile uint32_t g_imu_update_count = 0;
volatile uint32_t g_remote_update_count = 0;
volatile uint32_t g_motor_update_count = 0;

/* Double buffer for ISR→Task IMU data transfer */
static ICM42688_RawData_t g_imu_raw_buf[2];
static volatile uint32_t g_imu_raw_active_idx = 0;

/* ============================================================================
 *                          IMU ISR GLUE CODE
 * ============================================================================ */

/**
  * @brief Start TIM2 for precise 1 kHz IMU read interrupts
  * @note  TIM2 on APB1: 84 MHz / 84 = 1 MHz → / 1000 = 1 kHz
  *        NVIC priority: 5 (max syscall priority, allows FreeRTOS API in ISR)
  */
void IMU_StartTimerInterrupt(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM2->PSC = 84 - 1;
    TIM2->ARR = 1000 - 1;
    TIM2->DIER |= TIM_DIER_UIE;

    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief IMU ISR handler — called from TIM2_IRQHandler
  * @note  Reads raw IMU data via register-level SPI, stores in double buffer,
  *        then notifies Task_IMU via task notification.
  */
void IMU_ISR_Handler(void)
{
    ICM42688_RawData_t raw;

    if (ICM42688_ReadRawData_FromISR(&raw)) {
        uint32_t idx = g_imu_raw_active_idx;
        g_imu_raw_buf[idx] = raw;
        g_imu_raw_active_idx ^= 1;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(taskHandle_IMU, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief Get the latest raw data from the double buffer
  * @note  Reads the buffer that ISR is NOT currently writing to
  * @return Copy of the latest raw data
  */
ICM42688_RawData_t IMU_GetLatestRawData(void)
{
    return g_imu_raw_buf[g_imu_raw_active_idx ^ 1];
}

/* ============================================================================
 *                          TASK IMPLEMENTATIONS
 * ============================================================================ */

/**
  * @brief IMU data processing task
  * @note  Raw data is read by TIM2 ISR (hardware-timed 1kHz).
  *        This task waits for ISR notification, then scales/filters the data.
  */
void Task_IMU(void *argument)
{
    IMU_Data_t imuData;

    (void)argument;

    /* Initialize IMU (task context: HAL SPI with proper timeout) */
    if (!ICM42688_Init()) {
        g_system_status |= 0x01;  /* IMU init failed */
        vTaskSuspend(NULL);       /* Suspend this task */
    }

    /* Start TIM2 hardware timer for precise 1 kHz IMU reads */
    IMU_StartTimerInterrupt();

    for (;;) {
        /* Wait for TIM2 ISR notification — raw data is ready */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Read latest raw data from double buffer (ISR-safe) */
        ICM42688_RawData_t raw = IMU_GetLatestRawData();

        /* Scale, filter, update debug variables */
        ICM42688_ProcessRawData(&raw);

        /* Fill IMU data structure from processed debug vars */
        imuData.accel_x_g = g_imu_accel_x_filtered;
        imuData.accel_y_g = g_imu_accel_y_filtered;
        imuData.accel_z_g = g_imu_accel_z_filtered;
        imuData.gyro_x_dps = g_imu_gyro_x_filtered;
        imuData.gyro_y_dps = g_imu_gyro_y_filtered;
        imuData.gyro_z_dps = g_imu_gyro_z_filtered;
        imuData.temperature_c = g_imu_temperature_c;
        imuData.timestamp = osKernelGetTickCount();

        /* Send IMU data to queue (don't block if queue full) */
        osMessageQueuePut(queue_IMUData, &imuData, 0, 0);

        /* Signal IMU data ready */
        osSemaphoreRelease(sem_IMU_Ready);

        /* Update counter */
        g_imu_update_count++;
    }
}

/**
  * @brief Balance control task (high priority)
  * @note  Runs at 500Hz, implements balance algorithm
  */
void Task_Balance(void *argument)
{
    IMU_Data_t imuData;
    MotorCmd_t motorCmd;
    uint32_t tick_start;
    osStatus_t status;

    (void)argument;

    tick_start = osKernelGetTickCount();

    for (;;) {
        /* Wait for IMU data (max 2ms timeout) */
        status = osMessageQueueGet(queue_IMUData, &imuData, NULL, 2);

        if (status == osOK) {
            /* TODO: Implement balance control algorithm here */
            /* Example: PID control based on IMU data */

            /* Prepare motor commands */
            motorCmd.motor_id = 1;
            motorCmd.position = 0.0f;      /* Target position */
            motorCmd.velocity = 0.0f;      /* Target velocity */
            motorCmd.torque = 0.0f;        /* Target torque */
            motorCmd.mode = 0;             /* MIT mode */
            motorCmd.timestamp = osKernelGetTickCount();

            /* Send motor command to queue */
            osMessageQueuePut(queue_MotorCmd, &motorCmd, 0, 0);
        }

        /* Delay until next cycle (500Hz) */
        osDelayUntil(tick_start + 2);
        tick_start += 2;
    }
}

/**
  * @brief Motor control task (medium priority)
  * @note  Runs at 100Hz, sends commands to motors via CAN
  */
void Task_Motor(void *argument)
{
    MotorCmd_t motorCmd;
    uint32_t tick_start;
    osStatus_t status;

    (void)argument;

    tick_start = osKernelGetTickCount();

    for (;;) {
        /* Wait for motor command (max 10ms timeout) */
        status = osMessageQueueGet(queue_MotorCmd, &motorCmd, NULL, 10);

        if (status == osOK) {
            /* Lock CAN mutex */
            osMutexAcquire(mutex_CAN, osWaitForever);

            /* TODO: Send motor command via CAN */
            /* Example: EL05_MitControl(&motor, &cmd); */

            /* Unlock CAN mutex */
            osMutexRelease(mutex_CAN);

            /* Update counter */
            g_motor_update_count++;
        }

        /* Delay until next cycle (100Hz) */
        osDelayUntil(tick_start + 10);
        tick_start += 10;
    }
}

/**
  * @brief Remote control task (medium priority)
  * @note  Runs at 100Hz, reads data from NRF24L01
  */
void Task_Remote(void *argument)
{
    RemoteData_t remoteData;
    RemoteControlData_t *rc;
    uint32_t tick_start;

    (void)argument;

    /* Initialize NRF24L01 */
    NRF24L01_RX_Init();

    /* Wait for pairing (10s timeout) */
    if (!NRF24L01_RX_WaitForPairing()) {
        g_system_status |= 0x02;  /* Remote pairing failed */
    }

    tick_start = osKernelGetTickCount();

    for (;;) {
        /* Read remote data */
        if (NRF24L01_RX_ReadData()) {
            rc = NRF24L01_RX_GetData();

            /* Fill remote data structure */
            remoteData.right_x = (int16_t)rc->right_joystick_x - 128;
            remoteData.right_y = (int16_t)rc->right_joystick_y - 128;
            remoteData.left_x = (int16_t)rc->left_joystick_x - 128;
            remoteData.left_y = (int16_t)rc->left_joystick_y - 128;
            remoteData.buttons = rc->button_state;
            remoteData.online = NRF24L01_RX_IsOnline();
            remoteData.timestamp = osKernelGetTickCount();

            /* Send remote data to queue */
            osMessageQueuePut(queue_RemoteData, &remoteData, 0, 0);

            /* Signal remote data ready */
            osSemaphoreRelease(sem_Remote_Ready);

            /* Update counter */
            g_remote_update_count++;
        }

        /* Delay until next cycle (100Hz) */
        osDelayUntil(tick_start + 10);
        tick_start += 10;
    }
}

/**
  * @brief System monitoring task (low priority)
  * @note  Runs at 10Hz, monitors system status
  */
void Task_Monitor(void *argument)
{
    uint32_t tick_start;

    (void)argument;

    tick_start = osKernelGetTickCount();

    for (;;) {
        /* Check system status */
        /* TODO: Add safety checks */

        /* Check remote online status */
        if (!NRF24L01_RX_IsOnline()) {
            /* Remote offline - trigger safety mode */
            g_system_status |= 0x04;
        }

        /* Check IMU status */
        if (g_imu_update_count == 0) {
            /* IMU not updating */
            g_system_status |= 0x08;
        }

        /* Delay until next cycle (10Hz) */
        osDelayUntil(tick_start + 100);
        tick_start += 100;
    }
}

/**
  * @brief Debug output task (lowest priority)
  * @note  Runs at 1Hz, outputs debug information
  */
void Task_Debug(void *argument)
{
    uint32_t tick_start;

    (void)argument;

    tick_start = osKernelGetTickCount();

    for (;;) {
        /* Output debug information */
        /* TODO: Add debug output via UART */

        /* Delay until next cycle (1Hz) */
        osDelayUntil(tick_start + 1000);
        tick_start += 1000;
    }
}

/* ============================================================================
 *                          INITIALIZATION FUNCTIONS
 * ============================================================================ */

/**
  * @brief Create FreeRTOS tasks
  */
void FREERTOS_CreateTasks(void)
{
    /* Create queues */
    queue_IMUData = osMessageQueueNew(QUEUE_SIZE_IMU_DATA, sizeof(IMU_Data_t), NULL);
    queue_RemoteData = osMessageQueueNew(QUEUE_SIZE_REMOTE_DATA, sizeof(RemoteData_t), NULL);
    queue_MotorCmd = osMessageQueueNew(QUEUE_SIZE_MOTOR_CMD, sizeof(MotorCmd_t), NULL);

    /* Create mutexes */
    mutex_CAN = osMutexNew(NULL);
    mutex_SPI1 = osMutexNew(NULL);
    mutex_SPI3 = osMutexNew(NULL);

    /* Create semaphores */
    sem_IMU_Ready = osSemaphoreNew(1, 0, NULL);
    sem_Remote_Ready = osSemaphoreNew(1, 0, NULL);

    /* Create tasks */
    const osThreadAttr_t attr_IMU = {
        .name = "IMU_Task",
        .stack_size = TASK_STACK_SIZE_IMU * 4,
        .priority = TASK_PRIORITY_IMU_PROCESS
    };
    taskHandle_IMU = osThreadNew(Task_IMU, NULL, &attr_IMU);

    const osThreadAttr_t attr_Balance = {
        .name = "Balance_Task",
        .stack_size = TASK_STACK_SIZE_BALANCE * 4,
        .priority = TASK_PRIORITY_BALANCE
    };
    taskHandle_Balance = osThreadNew(Task_Balance, NULL, &attr_Balance);

    const osThreadAttr_t attr_Motor = {
        .name = "Motor_Task",
        .stack_size = TASK_STACK_SIZE_MOTOR * 4,
        .priority = TASK_PRIORITY_MOTOR
    };
    taskHandle_Motor = osThreadNew(Task_Motor, NULL, &attr_Motor);

    const osThreadAttr_t attr_Remote = {
        .name = "Remote_Task",
        .stack_size = TASK_STACK_SIZE_REMOTE * 4,
        .priority = TASK_PRIORITY_REMOTE
    };
    taskHandle_Remote = osThreadNew(Task_Remote, NULL, &attr_Remote);

    const osThreadAttr_t attr_Monitor = {
        .name = "Monitor_Task",
        .stack_size = TASK_STACK_SIZE_MONITOR * 4,
        .priority = TASK_PRIORITY_MONITOR
    };
    taskHandle_Monitor = osThreadNew(Task_Monitor, NULL, &attr_Monitor);

    const osThreadAttr_t attr_Debug = {
        .name = "Debug_Task",
        .stack_size = TASK_STACK_SIZE_DEBUG * 4,
        .priority = TASK_PRIORITY_DEBUG
    };
    taskHandle_Debug = osThreadNew(Task_Debug, NULL, &attr_Debug);
}

/**
  * @brief Initialize FreeRTOS
  */
void FREERTOS_Init(void)
{
    /* Initialize CMSIS-RTOS2 */
    osKernelInitialize();

    /* Create tasks */
    FREERTOS_CreateTasks();

    /* Start kernel */
    osKernelStart();
}