/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos.c
  * @brief   FreeRTOS task implementations for wheel-legged robot
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "icm42688.h"
#include "nrf24l01_rx.h"
#include "freertos_tasks.h"
#include "motor_driver.h"
#include "el05_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* ===== Task handles ===== */
osThreadId_t taskHandle_IMU = NULL;
osThreadId_t taskHandle_Balance = NULL;
osThreadId_t taskHandle_Motor = NULL;
osThreadId_t taskHandle_Remote = NULL;
osThreadId_t taskHandle_Monitor = NULL;
osThreadId_t taskHandle_Debug = NULL;

/* ===== Queue handles ===== */
osMessageQueueId_t queue_IMUData = NULL;
osMessageQueueId_t queue_RemoteData = NULL;
osMessageQueueId_t queue_MotorCmd = NULL;

/* ===== Mutex handles ===== */
osMutexId_t mutex_CAN = NULL;
osMutexId_t mutex_SPI1 = NULL;
osMutexId_t mutex_SPI3 = NULL;

/* ===== Semaphore handles ===== */
osSemaphoreId_t sem_IMU_Ready = NULL;
osSemaphoreId_t sem_Remote_Ready = NULL;

/* ===== Status variables ===== */
volatile uint8_t g_system_status = 0;
volatile uint32_t g_imu_update_count = 0;
volatile uint32_t g_remote_update_count = 0;

/* ===== Double buffer for ISR→Task IMU data transfer ===== */
static ICM42688_RawData_t g_imu_raw_buf[2];
static volatile uint32_t g_imu_raw_active_idx = 0;

/* ===== Unique to this file ===== */
osThreadId_t taskHandle_EL05_Motor;       /* EL05 joint motor task */
osThreadId_t taskHandle_M0601C_Motor;     /* M0601C wheel motor task */
osThreadId_t taskHandle_CAN;              /* CAN communication task */

osMessageQueueId_t queue_EL05_MotorCmd;   /* EL05 motor command queue */
osMessageQueueId_t queue_M0601C_MotorCmd; /* M0601C motor command queue */
osMessageQueueId_t queue_CAN_TX;          /* CAN TX message queue */
osMessageQueueId_t queue_CAN_RX;          /* CAN RX message queue */

osMutexId_t mutex_UART1;                  /* Protect UART1 (M0601C) */

osSemaphoreId_t sem_CAN_RX;               /* CAN RX complete */

volatile uint32_t g_el05_motor_update_count = 0;
volatile uint32_t g_m0601c_motor_update_count = 0;
volatile uint32_t g_can_tx_count = 0;
volatile uint32_t g_can_rx_count = 0;

/* External motor handles */
extern EL05_MotorHandle_t motor1;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* Unique tasks (only defined in this file) */
void Task_EL05_Motor(void *argument);
void Task_M0601C_Motor(void *argument);
void Task_CAN(void *argument);
/* Other tasks (Task_IMU, Task_Remote, etc.) are declared in freertos_tasks.h */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  mutex_CAN = osMutexNew(NULL);
  mutex_SPI1 = osMutexNew(NULL);
  mutex_SPI3 = osMutexNew(NULL);
  mutex_UART1 = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  sem_IMU_Ready = osSemaphoreNew(1, 0, NULL);
  sem_Remote_Ready = osSemaphoreNew(1, 0, NULL);
  sem_CAN_RX = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  queue_IMUData = osMessageQueueNew(10, sizeof(IMU_Data_t), NULL);
  queue_RemoteData = osMessageQueueNew(5, sizeof(RemoteData_t), NULL);
  queue_EL05_MotorCmd = osMessageQueueNew(10, sizeof(MotorCmd_t), NULL);
  queue_M0601C_MotorCmd = osMessageQueueNew(10, sizeof(MotorCmd_t), NULL);
  queue_CAN_TX = osMessageQueueNew(20, sizeof(CAN_TxHeaderTypeDef), NULL);
  queue_CAN_RX = osMessageQueueNew(20, sizeof(CAN_RxHeaderTypeDef), NULL);
  queue_MotorCmd = osMessageQueueNew(10, sizeof(MotorCmd_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  /* defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes); */

  /* USER CODE BEGIN RTOS_THREADS */
  /* IMU data processing task - ICM-42688-P (1kHz, triggered by TIM2 ISR) */
  {
    const osThreadAttr_t attr = {
      .name = "IMU_Task",
      .stack_size = 1024,
      .priority = (osPriority_t) osPriorityAboveNormal,
    };
    taskHandle_IMU = osThreadNew(Task_IMU, NULL, &attr);
  }

  /* Remote controller task - NRF24L01+ (100Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "Remote_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityAboveNormal1,
    };
    taskHandle_Remote = osThreadNew(Task_Remote, NULL, &attr);
  }

  /* EL05 joint motor task (100Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "EL05_Motor_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityAboveNormal,
    };
    taskHandle_EL05_Motor = osThreadNew(Task_EL05_Motor, NULL, &attr);
  }

  /* M0601C wheel motor task (50Hz) - reads remote joystick → drives wheels */
  {
    const osThreadAttr_t attr = {
      .name = "M0601C_Motor_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityAboveNormal,
    };
    taskHandle_M0601C_Motor = osThreadNew(Task_M0601C_Motor, NULL, &attr);
  }

  /* CAN communication task (500Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "CAN_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityAboveNormal2,
    };
    taskHandle_CAN = osThreadNew(Task_CAN, NULL, &attr);
  }

  /* Balance control task (500Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "Balance_Task",
      .stack_size = 4096,
      .priority = (osPriority_t) osPriorityAboveNormal2,
    };
    taskHandle_Balance = osThreadNew(Task_Balance, NULL, &attr);
  }

  /* System monitor task (10Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "Monitor_Task",
      .stack_size = 1024,
      .priority = (osPriority_t) osPriorityBelowNormal,
    };
    taskHandle_Monitor = osThreadNew(Task_Monitor, NULL, &attr);
  }

  /* Debug output task (1Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "Debug_Task",
      .stack_size = 1024,
      .priority = (osPriority_t) osPriorityLow,
    };
    taskHandle_Debug = osThreadNew(Task_Debug, NULL, &attr);
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief EL05 motor — MIT torque control via private protocol type 1
  */
void Task_EL05_Motor(void *argument)
{
    EL05_MitControl_t mit;
    uint32_t tick;
    uint8_t initialized = 0;

    (void)argument;

    osDelay(200);
    tick = osKernelGetTickCount();

    for (;;)
    {
        if (!initialized) {
            /* Set run_mode = 0 (MIT/运控模式) */
            osMutexAcquire(mutex_CAN, osWaitForever);
            EL05_WriteParamU8(&motor1, 0x7005, 0);
            osMutexRelease(mutex_CAN);
            osDelay(5);

            /* Disable motor — rotation logic temporarily stopped for ID setup */
            osMutexAcquire(mutex_CAN, osWaitForever);
            EL05_Disable(&motor1);
            osMutexRelease(mutex_CAN);
            osDelay(10);

            /* Write current limit */
            osMutexAcquire(mutex_CAN, osWaitForever);
            EL05_WriteParam(&motor1, 0x7018, 5.0f);
            osMutexRelease(mutex_CAN);

            initialized = 1;
        }

        /* MIT control paused — all outputs zeroed */
        mit.p_des = 0.0f;
        mit.v_des = 0.0f;
        mit.kp    = 0.0f;
        mit.kd    = 0.0f;
        mit.t_ff  = 0.0f;

        osMutexAcquire(mutex_CAN, osWaitForever);
        EL05_MitControl(&motor1, &mit);
        osMutexRelease(mutex_CAN);

        g_el05_motor_update_count++;
        osDelayUntil(tick + 10);
        tick += 10;
    }
}

/**
  * @brief CAN debug variables
  */
volatile uint32_t g_can_esr = 0;
volatile uint32_t g_can_tsr = 0;

/**
  * @brief Raw CAN RX capture (updated for EVERY received frame)
  */
volatile uint32_t g_can_rx_raw_id = 0;
volatile uint8_t  g_can_rx_raw_ide = 0;
volatile uint8_t  g_can_rx_raw_dlc = 0;
volatile uint8_t  g_can_rx_raw_data0 = 0;

/**
  * @brief Motor feedback capture (updated by CAN RX callback)
  */
volatile int16_t g_motor_fb_pos_int = 0;
volatile int16_t g_motor_fb_vel_int = 0;
volatile int16_t g_motor_fb_trq_int = 0;
volatile uint16_t g_motor_fb_temp_int = 0;
volatile uint8_t  g_motor_fb_fault = 0;
volatile uint8_t  g_motor_fb_id = 0;
volatile uint8_t  g_motor_fb_mode_state = 0;

/**
  * @brief M0601C wheel motor control task (50Hz)
  * @note  Reads remote joystick from queue_RemoteData,
  *        maps left Y-axis to RPM, drives both M0601C wheel motors.
  */
void Task_M0601C_Motor(void *argument)
{
    RemoteData_t rc;
    osStatus_t status;
    uint32_t tick;

    (void)argument;

    /* --- Configurable parameters --- */
    #define WHEEL_RPM_MAX       200      /* Max wheel speed (RPM) */
    #define WHEEL_DEADBAND      8        /* Joystick deadband (±8) */
    #define MOTOR_ID_LEFT       1        /* Left wheel motor ID */
    #define MOTOR_ID_RIGHT      2        /* Right wheel motor ID */

    /* --- Initialize --- */
    MOTOR_StartReceive();

    /*
     * Ensure motors are in speed mode.
     * On first power-up, motors default to speed mode, so this is optional.
     * Uncomment if motors were left in position/current mode:
     *
     *   osMutexAcquire(mutex_UART1, osWaitForever);
     *   MOTOR_SendModeSwitchCmd(MOTOR_ID_LEFT,  MOTOR_CTRL_SPEED);
     *   MOTOR_SendModeSwitchCmd(MOTOR_ID_RIGHT, MOTOR_CTRL_SPEED);
     *   osMutexRelease(mutex_UART1);
     */

    tick = osKernelGetTickCount();

    for (;;)
    {
        /* --- 1. Get latest remote data (non-blocking, 20ms timeout) --- */
        status = osMessageQueueGet(queue_RemoteData, &rc, NULL, 20);

        if (status == osOK) {
            /* --- 2. Map left Y-axis to RPM --- */
            /*
             * rc.left_y: -128 (forward) … 0 (center) … 127 (backward)
             *
             *    forward (negative):  rpm = +max   (wheel spins forward)
             *    backward (positive): rpm = -max   (wheel spins reverse)
             *    center  (±deadband): rpm = 0
             */
            int32_t raw = rc.left_y;
            int16_t rpm_left, rpm_right;

            /* Apply deadband */
            if (raw > -WHEEL_DEADBAND && raw < WHEEL_DEADBAND) {
                rpm_left  = 0;
                rpm_right = 0;
            } else {
                /* Map -128..-9 → +max..+1,  +9..+127 → -1..-max */
                int32_t scaled;
                if (raw < 0) {
                    scaled = (raw + WHEEL_DEADBAND) * WHEEL_RPM_MAX / (128 - WHEEL_DEADBAND);
                    rpm_left  = (int16_t)(-scaled);   /* forward */
                    rpm_right = (int16_t)(-scaled);
                } else {
                    scaled = (raw - WHEEL_DEADBAND) * WHEEL_RPM_MAX / (127 - WHEEL_DEADBAND);
                    rpm_left  = (int16_t)(-scaled);   /* backward */
                    rpm_right = (int16_t)(-scaled);
                }
            }

            /* --- 3. Send speed commands to both wheel motors --- */
            osMutexAcquire(mutex_UART1, osWaitForever);
            MOTOR_SetSpeed(MOTOR_ID_LEFT,  rpm_left);
            MOTOR_SetSpeed(MOTOR_ID_RIGHT, rpm_right);
            osMutexRelease(mutex_UART1);

        } else {
            /* --- 4. No remote data — safety stop --- */
            osMutexAcquire(mutex_UART1, osWaitForever);
            MOTOR_Stop(MOTOR_ID_LEFT);
            MOTOR_Stop(MOTOR_ID_RIGHT);
            osMutexRelease(mutex_UART1);
        }

        g_m0601c_motor_update_count++;
        osDelayUntil(tick + 20);  /* 50Hz */
        tick += 20;
    }
}

/**
  * @brief CAN communication management task (500Hz)
  * @note  Handles CAN TX/RX for EL05 motors
  */
void Task_CAN(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* Sample CAN error/status registers for debugging */
        if (hcan1.Instance) {
            g_can_esr = hcan1.Instance->ESR;   /* Error Status Register */
            g_can_tsr = hcan1.Instance->TSR;   /* Transmit Status Register */
        }

        g_can_tx_count++;
        g_can_rx_count++;
        osDelayUntil(tick + 2);
        tick += 2;
    }
}

/**
  * @brief Stack overflow hook function
  * @note  Called when stack overflow is detected
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    /* Stack overflow detected - enter infinite loop for debugging */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

/* ============================================================================
 *                          IMU ISR GLUE CODE
 * ============================================================================ */

/**
  * @brief Start TIM2 for precise 1 kHz IMU read interrupts
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
  */
void Task_IMU(void *argument)
{
    IMU_Data_t imuData;

    (void)argument;

    if (!ICM42688_Init()) {
        g_system_status |= 0x01;
        vTaskSuspend(NULL);
    }

    IMU_StartTimerInterrupt();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ICM42688_RawData_t raw = IMU_GetLatestRawData();
        ICM42688_ProcessRawData(&raw);

        imuData.accel_x_g = g_imu_accel_x_filtered;
        imuData.accel_y_g = g_imu_accel_y_filtered;
        imuData.accel_z_g = g_imu_accel_z_filtered;
        imuData.gyro_x_dps = g_imu_gyro_x_filtered;
        imuData.gyro_y_dps = g_imu_gyro_y_filtered;
        imuData.gyro_z_dps = g_imu_gyro_z_filtered;
        imuData.temperature_c = g_imu_temperature_c;
        imuData.timestamp = osKernelGetTickCount();

        osMessageQueuePut(queue_IMUData, &imuData, 0, 0);
        osSemaphoreRelease(sem_IMU_Ready);

        g_imu_update_count++;
    }
}

/**
  * @brief Balance control task (high priority, 500Hz)
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
        status = osMessageQueueGet(queue_IMUData, &imuData, NULL, 2);

        if (status == osOK) {
            motorCmd.motor_id = 1;
            motorCmd.position = 0.0f;
            motorCmd.velocity = 0.0f;
            motorCmd.torque = 0.0f;
            motorCmd.mode = 0;
            motorCmd.timestamp = osKernelGetTickCount();

            osMessageQueuePut(queue_MotorCmd, &motorCmd, 0, 0);
        }

        osDelayUntil(tick_start + 2);
        tick_start += 2;
    }
}

/**
  * @brief Remote control task (100Hz)
  */
void Task_Remote(void *argument)
{
    RemoteData_t remoteData;
    RemoteControlData_t *rc;
    uint32_t tick_start;

    (void)argument;

    NRF24L01_RX_Init();

    if (!NRF24L01_RX_WaitForPairing()) {
        g_system_status |= 0x02;
    }

    tick_start = osKernelGetTickCount();

    for (;;) {
        if (NRF24L01_RX_ReadData()) {
            rc = NRF24L01_RX_GetData();

            remoteData.right_x = (int16_t)rc->right_joystick_x - 128;
            remoteData.right_y = (int16_t)rc->right_joystick_y - 128;
            remoteData.left_x = (int16_t)rc->left_joystick_x - 128;
            remoteData.left_y = (int16_t)rc->left_joystick_y - 128;
            remoteData.buttons = rc->button_state;
            remoteData.online = NRF24L01_RX_IsOnline();
            remoteData.timestamp = osKernelGetTickCount();

            osMessageQueuePut(queue_RemoteData, &remoteData, 0, 0);
            osSemaphoreRelease(sem_Remote_Ready);

            g_remote_update_count++;
        }

        osDelayUntil(tick_start + 10);
        tick_start += 10;
    }
}

/**
  * @brief System monitoring task (10Hz)
  */
void Task_Monitor(void *argument)
{
    uint32_t tick_start;

    (void)argument;

    tick_start = osKernelGetTickCount();

    for (;;) {
        if (!NRF24L01_RX_IsOnline()) {
            g_system_status |= 0x04;
        }

        if (g_imu_update_count == 0) {
            g_system_status |= 0x08;
        }

        osDelayUntil(tick_start + 100);
        tick_start += 100;
    }
}

/**
  * @brief Debug output task (1Hz)
  */
void Task_Debug(void *argument)
{
    uint32_t tick_start;

    (void)argument;

    tick_start = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(tick_start + 1000);
        tick_start += 1000;
    }
}

/* USER CODE END Application */

/* FREERTOS_END_OF_FILE */
