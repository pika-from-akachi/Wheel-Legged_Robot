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
#include "icm42688.h"
#include "nrf24l01_rx.h"
#include "freertos_tasks.h"
#include "motor_driver.h"
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
/* ===== Shared variables (defined in freertos_tasks.c) ===== */
extern osThreadId_t taskHandle_IMU;
extern osThreadId_t taskHandle_Remote;
extern osThreadId_t taskHandle_Balance;
extern osThreadId_t taskHandle_Monitor;
extern osThreadId_t taskHandle_Debug;
extern osMessageQueueId_t queue_IMUData;
extern osMessageQueueId_t queue_RemoteData;
extern osMutexId_t mutex_CAN;
extern osMutexId_t mutex_SPI1;
extern osMutexId_t mutex_SPI3;
extern osSemaphoreId_t sem_IMU_Ready;
extern osSemaphoreId_t sem_Remote_Ready;
extern volatile uint8_t g_system_status;
extern volatile uint32_t g_imu_update_count;
extern volatile uint32_t g_remote_update_count;

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
  * @brief EL05 joint motor control task (100Hz)
  * @note  Controls EL05 motors via CAN extended frame
  */
void Task_EL05_Motor(void *argument)
{
    (void)argument;

    /* TODO: Initialize EL05 motor driver */
    /* EL05_Init(&hcan1); */
    /* EL05_StartReception(); */

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* TODO: Receive motor command from queue */
        /* if (osMessageQueueGet(queue_EL05_MotorCmd, &motorCmd, NULL, 10) == osOK) {
            osMutexAcquire(mutex_CAN, osWaitForever);
            EL05_MitControl(&motor, &cmd);
            osMutexRelease(mutex_CAN);
        } */

        g_el05_motor_update_count++;
        osDelayUntil(tick + 10);
        tick += 10;
    }
}

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

    /* TODO: Initialize CAN */
    /* HAL_CAN_Start(&hcan1); */
    /* HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); */

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* TODO: Process CAN TX queue */
        /* osMutexAcquire(mutex_CAN, osWaitForever); */
        /* Process CAN messages */
        /* osMutexRelease(mutex_CAN); */

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

/* USER CODE END Application */
