/**
  ******************************************************************************
  * @file    freertos_tasks.c
  * @brief   FreeRTOS task implementations for wheel-legged robot
  ******************************************************************************
  */

#include "freertos_tasks.h"
#include "icm42688.h"
#include "nrf24l01_rx.h"
#include "el05_motor.h"
#include "m0601c_motor.h"
#include "lqr_control.h"
#include "esp32_com.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 *                          MOTOR ID MAPPING
 * ============================================================================
 *  EL05 joint motors: IDs 1-4 (CAN)
 *  M0601C hub motors: IDs 5-6 (RS485, mapped from RS485 IDs 1-2)
 * ============================================================================ */

#define M0601C_QUEUE_ID_OFFSET   4        /* M0601C starts at queue ID 5 */
#define M0601C_WHEEL_TORQUE_TO_CURRENT  4.096f / 0.15f  /* Raw = Nm / Kt * (32767/8000), Kt≈0.15 */

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
osThreadId_t taskHandle_LQR = NULL;
osThreadId_t taskHandle_Motor = NULL;
osThreadId_t taskHandle_Remote = NULL;
osThreadId_t taskHandle_ESP32_COM = NULL;
osThreadId_t taskHandle_Monitor = NULL;
osThreadId_t taskHandle_Debug = NULL;

/* Queue handles */
osMessageQueueId_t queue_IMUData = NULL;
osMessageQueueId_t queue_RemoteData = NULL;
osMessageQueueId_t queue_MotorCmd = NULL;
osMessageQueueId_t queue_ESP32Cmd = NULL;

/* Mutex handles */
osMutexId_t mutex_CAN = NULL;
osMutexId_t mutex_SPI1 = NULL;
osMutexId_t mutex_SPI3 = NULL;
osMutexId_t mutex_UART_ESP32 = NULL;
osMutexId_t mutex_RS485 = NULL;

/* Semaphore handles */
osSemaphoreId_t sem_IMU_Ready = NULL;
osSemaphoreId_t sem_Remote_Ready = NULL;

/* Status variables */
volatile uint8_t g_system_status = 0;
volatile uint32_t g_imu_update_count = 0;
volatile uint32_t g_remote_update_count = 0;
volatile uint32_t g_motor_update_count = 0;
volatile uint32_t g_lqr_update_count = 0;
volatile uint32_t g_esp32_packet_count = 0;

/* Global LQR controller and tuning params */
LQR_Controller_t g_lqr_controller;
LQR_TuningParams_t g_lqr_tuning;
RobotState_t g_robot_state;

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
            /* Update robot state from IMU */
            g_robot_state.dq_body = imuData.gyro_x_dps * 0.017453f; /* deg/s to rad/s */

            /* Estimate body angle from accel (complementary filter) */
            static float body_angle = 0.0f;
            float accel_angle = atan2f(imuData.accel_y_g, imuData.accel_z_g);
            float gyro_rate = imuData.gyro_x_dps * 0.017453f;
            static uint32_t last_tick = 0;
            float dt = (osKernelGetTickCount() - last_tick) * 0.001f;
            if (dt > 0.001f && dt < 0.1f) {
                body_angle = 0.98f * (body_angle + gyro_rate * dt) + 0.02f * accel_angle;
            }
            last_tick = osKernelGetTickCount();
            g_robot_state.q_body = body_angle;

            /* Send motor commands for balance */
            if (g_lqr_controller.enabled) {
                BalanceState_t bal = ROBOT_ExtractBalanceState(&g_robot_state);
                LQR_Update(&g_lqr_controller, &bal, 0.002f);

                /* Distribute LQR output to 4 joint motors */
                float joint_torque = g_lqr_controller.u[0];
                float wheel_torque = g_lqr_controller.u[1];

                /* Left side */
                motorCmd.motor_id = ROBOT_EL05_ID_HIP_L;
                motorCmd.position = 0.0f;
                motorCmd.velocity = 0.0f;
                motorCmd.torque = joint_torque * 0.5f;
                motorCmd.mode = EL05_MODE_MIT;
                motorCmd.timestamp = osKernelGetTickCount();
                osMessageQueuePut(queue_MotorCmd, &motorCmd, 0, 0);

                motorCmd.motor_id = ROBOT_EL05_ID_KNEE_L;
                motorCmd.torque = joint_torque * 0.5f;
                osMessageQueuePut(queue_MotorCmd, &motorCmd, 0, 0);

                /* Right side */
                motorCmd.motor_id = ROBOT_EL05_ID_HIP_R;
                motorCmd.torque = joint_torque * 0.5f;
                osMessageQueuePut(queue_MotorCmd, &motorCmd, 0, 0);

                motorCmd.motor_id = ROBOT_EL05_ID_KNEE_R;
                motorCmd.torque = joint_torque * 0.5f;
                osMessageQueuePut(queue_MotorCmd, &motorCmd, 0, 0);

                /* ===== M0601C Wheel Motors: convert LQR wheel torque to current command ===== */
                int16_t wheel_current_raw = (int16_t)(wheel_torque * M0601C_WHEEL_TORQUE_TO_CURRENT);
                /* Clamp to valid range */
                if (wheel_current_raw > 32767) wheel_current_raw = 32767;
                if (wheel_current_raw < -32767) wheel_current_raw = -32767;

                motorCmd.motor_id = M0601C_QUEUE_ID_OFFSET + ROBOT_M0601C_ID_WHEEL_L;
                motorCmd.torque = wheel_torque * 0.5f;
                motorCmd.position = (float)wheel_current_raw;
                motorCmd.velocity = 0.0f;
                motorCmd.mode = M0601C_MODE_CURRENT;
                motorCmd.timestamp = osKernelGetTickCount();
                osMessageQueuePut(queue_MotorCmd, &motorCmd, 0, 0);

                motorCmd.motor_id = M0601C_QUEUE_ID_OFFSET + ROBOT_M0601C_ID_WHEEL_R;
                motorCmd.torque = wheel_torque * 0.5f;
                motorCmd.position = (float)wheel_current_raw;
                osMessageQueuePut(queue_MotorCmd, &motorCmd, 0, 0);
            }
        }

        /* Delay until next cycle (500Hz) */
        osDelayUntil(tick_start + 2);
        tick_start += 2;
    }
}

/**
  * @brief LQR control task (high priority)
  * @note  Runs at 1kHz, dedicated LQR computation and safety monitoring
  */
void Task_LQR(void *argument)
{
    BalanceState_t balance_state;
    uint32_t tick_start;

    (void)argument;

    /* Initialize LQR controller */
    LQR_Init(&g_lqr_controller);
    LQR_SetDefaultTuning(&g_lqr_tuning);
    LQR_ComputeGains(&g_lqr_tuning, &g_lqr_controller.gain);
    ROBOT_Init();
    ROBOT_ResetState(&g_robot_state);

    tick_start = osKernelGetTickCount();

    for (;;) {
        /* Extract balance state from robot state */
        balance_state = ROBOT_ExtractBalanceState(&g_robot_state);

        /* Safety check */
        if (!LQR_CheckSafety(&g_lqr_controller, &balance_state)) {
            if (g_lqr_controller.enabled) {
                LQR_EmergencyStop(&g_lqr_controller);
                g_system_status |= 0x10; /* LQR safety trigger */
            }
        }

        /* Process ESP32 packets (non-blocking) */
        ESP32_COM_Update();

        /* Update counters */
        g_lqr_update_count++;

        /* Delay until next cycle (1kHz) */
        osDelayUntil(tick_start + 1);
        tick_start += 1;
    }
}

/**
  * @brief Motor control task (medium priority)
  * @note  Runs at 200Hz, sends commands to all 6 motors (4 EL05 CAN + 2 M0601C RS485)
  */
void Task_Motor(void *argument)
{
    MotorCmd_t motorCmd;
    EL05_MitControl_t mit_cmd;
    uint32_t tick_start;
    osStatus_t status;

    (void)argument;

    tick_start = osKernelGetTickCount();

    /* Extern motor handles */
    extern EL05_MotorHandle_t g_el05_motors[4];
    extern M0601C_MotorHandle_t g_m0601c_motors[2];

    for (;;) {
        /* Process all pending motor commands */
        while (osMessageQueueGet(queue_MotorCmd, &motorCmd, NULL, 0) == osOK) {
            osMutexAcquire(mutex_CAN, osWaitForever);

            /* EL05 Joint Motors (CAN, IDs 1-4) */
            if (motorCmd.motor_id >= 1 && motorCmd.motor_id <= 4) {
                uint8_t idx = motorCmd.motor_id - 1;
                mit_cmd.p_des = motorCmd.position;
                mit_cmd.v_des = motorCmd.velocity;
                mit_cmd.kp = 20.0f;
                mit_cmd.kd = 0.5f;
                mit_cmd.t_ff = motorCmd.torque;
                EL05_MitControl(&g_el05_motors[idx], &mit_cmd);
            }

            osMutexRelease(mutex_CAN);
            g_motor_update_count++;
        }

        /* Process M0601C commands (non-blocking peek into queue) */
        osMutexAcquire(mutex_RS485, osWaitForever);
        while (osMessageQueueGet(queue_MotorCmd, &motorCmd, NULL, 0) == osOK) {
            if (motorCmd.motor_id >= (M0601C_QUEUE_ID_OFFSET + 1) &&
                motorCmd.motor_id <= (M0601C_QUEUE_ID_OFFSET + 2)) {
                uint8_t m0601c_idx = motorCmd.motor_id - M0601C_QUEUE_ID_OFFSET - 1;
                int16_t current_raw = (int16_t)motorCmd.position;
                M0601C_CurrentControl(&g_m0601c_motors[m0601c_idx], current_raw);
            }
        }

        /* Poll M0601C feedback (every cycle) */
        for (int i = 0; i < 2; i++) {
            M0601C_RequestFeedback(&g_m0601c_motors[i]);
            for (volatile uint32_t d = 0; d < 500; d++) {} /* ~50ns delay */
        }
        /* Process any received RS485 frames */
        M0601C_ProcessRxFrame(g_m0601c_motors, 2);
        osMutexRelease(mutex_RS485);

        /* Delay until next cycle (200Hz) */
        osDelayUntil(tick_start + 5);
        tick_start += 5;
    }
}

/**
  * @brief ESP32 communication task (medium-low priority)
  * @note  Runs at 100Hz, handles UART data exchange with ESP32-S3
  */
void Task_ESP32_COM(void *argument)
{
    uint32_t tick_start;

    (void)argument;

    /* Initialize ESP32 communication */
    extern UART_HandleTypeDef huart3;
    ESP32_COM_Init(&huart3);
    ESP32_COM_StartRx();

    tick_start = osKernelGetTickCount();

    for (;;) {
        /* Send state data to ESP32 every cycle (100Hz) */
        osMutexAcquire(mutex_UART_ESP32, osWaitForever);
        ESP32_COM_SendStateData();
        osMutexRelease(mutex_UART_ESP32);

        /* Send heartbeat every 100 cycles (~1 second) */
        static uint32_t hb_count = 0;
        if (++hb_count >= 100) {
            ESP32_COM_SendAck(ESP32_ERR_NONE);
            hb_count = 0;
        }

        /* Update counters */
        g_esp32_packet_count++;

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
    queue_ESP32Cmd = osMessageQueueNew(QUEUE_SIZE_ESP32_CMD, sizeof(uint8_t) * 20, NULL);

    /* Create mutexes */
    mutex_CAN = osMutexNew(NULL);
    mutex_SPI1 = osMutexNew(NULL);
    mutex_SPI3 = osMutexNew(NULL);
    mutex_UART_ESP32 = osMutexNew(NULL);
    mutex_RS485 = osMutexNew(NULL);

    /* Create semaphores */
    sem_IMU_Ready = osSemaphoreNew(1, 0, NULL);
    sem_Remote_Ready = osSemaphoreNew(1, 0, NULL);

    /* Create tasks */

    /* IMU task */
    const osThreadAttr_t attr_IMU = {
        .name = "IMU_Task",
        .stack_size = TASK_STACK_SIZE_IMU * 4,
        .priority = TASK_PRIORITY_IMU_PROCESS
    };
    taskHandle_IMU = osThreadNew(Task_IMU, NULL, &attr_IMU);

    /* LQR control task (1kHz) */
    const osThreadAttr_t attr_LQR = {
        .name = "LQR_Task",
        .stack_size = TASK_STACK_SIZE_LQR * 4,
        .priority = TASK_PRIORITY_LQR_CONTROL
    };
    taskHandle_LQR = osThreadNew(Task_LQR, NULL, &attr_LQR);

    /* Balance control task (500Hz) */
    const osThreadAttr_t attr_Balance = {
        .name = "Balance_Task",
        .stack_size = TASK_STACK_SIZE_BALANCE * 4,
        .priority = TASK_PRIORITY_BALANCE
    };
    taskHandle_Balance = osThreadNew(Task_Balance, NULL, &attr_Balance);

    /* Motor control task (200Hz) */
    const osThreadAttr_t attr_Motor = {
        .name = "Motor_Task",
        .stack_size = TASK_STACK_SIZE_MOTOR * 4,
        .priority = TASK_PRIORITY_MOTOR
    };
    taskHandle_Motor = osThreadNew(Task_Motor, NULL, &attr_Motor);

    /* Remote control task (100Hz) */
    const osThreadAttr_t attr_Remote = {
        .name = "Remote_Task",
        .stack_size = TASK_STACK_SIZE_REMOTE * 4,
        .priority = TASK_PRIORITY_REMOTE
    };
    taskHandle_Remote = osThreadNew(Task_Remote, NULL, &attr_Remote);

    /* ESP32 communication task (50Hz) */
    const osThreadAttr_t attr_ESP32 = {
        .name = "ESP32_COM_Task",
        .stack_size = TASK_STACK_SIZE_ESP32_COM * 4,
        .priority = TASK_PRIORITY_ESP32_COM
    };
    taskHandle_ESP32_COM = osThreadNew(Task_ESP32_COM, NULL, &attr_ESP32);

    /* Monitor task (10Hz) */
    const osThreadAttr_t attr_Monitor = {
        .name = "Monitor_Task",
        .stack_size = TASK_STACK_SIZE_MONITOR * 4,
        .priority = TASK_PRIORITY_MONITOR
    };
    taskHandle_Monitor = osThreadNew(Task_Monitor, NULL, &attr_Monitor);

    /* Debug task (1Hz) */
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