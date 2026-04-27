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
/* Task handles */
osThreadId_t taskHandle_IMU;              /* ICM-42688-P IMU task */
osThreadId_t taskHandle_Remote;           /* NRF24L01+ remote controller task */
osThreadId_t taskHandle_EL05_Motor;       /* EL05 joint motor task */
osThreadId_t taskHandle_M0601C_Motor;     /* M0601C wheel motor task */
osThreadId_t taskHandle_CAN;              /* CAN communication task */
osThreadId_t taskHandle_Balance;          /* Balance control task */
osThreadId_t taskHandle_Monitor;          /* System monitor task */
osThreadId_t taskHandle_Debug;            /* Debug output task */

/* Queue handles */
osMessageQueueId_t queue_IMUData;         /* IMU sensor data queue */
osMessageQueueId_t queue_RemoteData;      /* Remote control data queue */
osMessageQueueId_t queue_EL05_MotorCmd;   /* EL05 motor command queue */
osMessageQueueId_t queue_M0601C_MotorCmd; /* M0601C motor command queue */
osMessageQueueId_t queue_CAN_TX;          /* CAN TX message queue */
osMessageQueueId_t queue_CAN_RX;          /* CAN RX message queue */

/* Mutex handles */
osMutexId_t mutex_CAN;                    /* Protect CAN bus */
osMutexId_t mutex_SPI1;                   /* Protect SPI1 (IMU) */
osMutexId_t mutex_SPI3;                   /* Protect SPI3 (NRF24L01) */
osMutexId_t mutex_UART1;                  /* Protect UART1 (M0601C) */

/* Semaphore handles */
osSemaphoreId_t sem_IMU_Ready;            /* IMU data ready */
osSemaphoreId_t sem_Remote_Ready;         /* Remote data ready */
osSemaphoreId_t sem_CAN_RX;               /* CAN RX complete */

/* Status variables */
volatile uint8_t g_system_status = 0;
volatile uint32_t g_imu_update_count = 0;
volatile uint32_t g_remote_update_count = 0;
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
void Task_IMU(void *argument);
void Task_Remote(void *argument);
void Task_EL05_Motor(void *argument);
void Task_M0601C_Motor(void *argument);
void Task_CAN(void *argument);
void Task_Balance(void *argument);
void Task_Monitor(void *argument);
void Task_Debug(void *argument);
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
  queue_IMUData = osMessageQueueNew(10, sizeof(void*), NULL);
  queue_RemoteData = osMessageQueueNew(5, sizeof(void*), NULL);
  queue_EL05_MotorCmd = osMessageQueueNew(10, sizeof(void*), NULL);
  queue_M0601C_MotorCmd = osMessageQueueNew(10, sizeof(void*), NULL);
  queue_CAN_TX = osMessageQueueNew(20, sizeof(void*), NULL);
  queue_CAN_RX = osMessageQueueNew(20, sizeof(void*), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  /* defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes); */

  /* USER CODE BEGIN RTOS_THREADS */
  /* IMU task - ICM-42688-P (1kHz, highest priority) */
  {
    const osThreadAttr_t attr = {
      .name = "IMU_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityAboveNormal3,
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

  /* M0601C wheel motor task (100Hz) */
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
  * @brief ICM-42688-P IMU data acquisition task (1kHz)
  * @note  Reads IMU data with Kalman filter
  */
void Task_IMU(void *argument)
{
    (void)argument;

    /* TODO: Initialize ICM-42688-P */
    /* if (!ICM42688_Init()) {
        g_system_status |= 0x01;
        vTaskSuspend(NULL);
    } */

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* TODO: Read IMU data with Kalman filter */
        /* ICM42688_Update(); */

        /* TODO: Send IMU data to queue */
        /* osMessageQueuePut(queue_IMUData, &imuData, 0, 0); */

        g_imu_update_count++;
        osDelayUntil(tick + 1);
        tick += 1;
    }
}

/**
  * @brief NRF24L01+ remote controller task (100Hz)
  * @note  Reads remote control data
  */
void Task_Remote(void *argument)
{
    (void)argument;

    /* TODO: Initialize NRF24L01+ */
    /* NRF24L01_RX_Init(); */
    /* if (!NRF24L01_RX_WaitForPairing()) {
        g_system_status |= 0x02;
    } */

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* TODO: Read remote data */
        /* if (NRF24L01_RX_ReadData()) {
            RemoteControlData_t *rc = NRF24L01_RX_GetData();
            osMessageQueuePut(queue_RemoteData, &remoteData, 0, 0);
        } */

        g_remote_update_count++;
        osDelayUntil(tick + 10);
        tick += 10;
    }
}

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
  * @brief M0601C wheel motor control task (100Hz)
  * @note  Controls M0601C motors via UART
  */
void Task_M0601C_Motor(void *argument)
{
    (void)argument;

    /* TODO: Initialize M0601C motor driver */
    /* MOTOR_StartReceive(); */

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* TODO: Receive motor command from queue */
        /* if (osMessageQueueGet(queue_M0601C_MotorCmd, &motorCmd, NULL, 10) == osOK) {
            osMutexAcquire(mutex_UART1, osWaitForever);
            MOTOR_SetSpeed(motorCmd.motor_id, motorCmd.velocity);
            osMutexRelease(mutex_UART1);
        } */

        g_m0601c_motor_update_count++;
        osDelayUntil(tick + 10);
        tick += 10;
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
  * @brief Balance control task (500Hz)
  * @note  Implements balance algorithm using IMU and remote data
  */
void Task_Balance(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* TODO: Get IMU data */
        /* osMessageQueueGet(queue_IMUData, &imuData, NULL, 2); */

        /* TODO: Get remote data */
        /* osMessageQueueGet(queue_RemoteData, &remoteData, NULL, 0); */

        /* TODO: Implement balance control algorithm */
        /* Use IMU data and remote data to calculate motor commands */

        /* TODO: Send motor commands */
        /* osMessageQueuePut(queue_EL05_MotorCmd, &motorCmd, 0, 0); */
        /* osMessageQueuePut(queue_M0601C_MotorCmd, &motorCmd, 0, 0); */

        osDelayUntil(tick + 2);
        tick += 2;
    }
}

/**
  * @brief System monitor task (10Hz)
  * @note  Monitors system status and safety
  */
void Task_Monitor(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* TODO: Check remote online status */
        /* if (!NRF24L01_RX_IsOnline()) {
            g_system_status |= 0x04;
        } else {
            g_system_status &= ~0x04;
        } */

        /* TODO: Check IMU status */
        /* if (g_imu_update_count == 0) {
            g_system_status |= 0x08;
        } */

        /* TODO: Check motor status */

        /* TODO: Safety protection */
        /* if (g_system_status != 0) {
            Stop all motors
        } */

        osDelayUntil(tick + 100);
        tick += 100;
    }
}

/**
  * @brief Debug output task (1Hz)
  * @note  Outputs debug information via UART
  */
void Task_Debug(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* TODO: Output debug information */
        /* printf("IMU: %d, Remote: %d, EL05: %d, M0601C: %d\n",
               g_imu_update_count, g_remote_update_count,
               g_el05_motor_update_count, g_m0601c_motor_update_count); */

        osDelayUntil(tick + 1000);
        tick += 1000;
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
