/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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
osThreadId_t taskHandle_IMU;
osThreadId_t taskHandle_Balance;
osThreadId_t taskHandle_Motor;
osThreadId_t taskHandle_Remote;
osThreadId_t taskHandle_Monitor;
osThreadId_t taskHandle_Debug;

/* Queue handles */
osMessageQueueId_t queue_IMUData;
osMessageQueueId_t queue_RemoteData;
osMessageQueueId_t queue_MotorCmd;

/* Mutex handles */
osMutexId_t mutex_CAN;
osMutexId_t mutex_SPI1;
osMutexId_t mutex_SPI3;

/* Status variables */
volatile uint8_t g_system_status = 0;
volatile uint32_t g_imu_update_count = 0;
volatile uint32_t g_remote_update_count = 0;
volatile uint32_t g_motor_update_count = 0;
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
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  /* defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes); */

  /* USER CODE BEGIN RTOS_THREADS */
  /* IMU task - highest priority (1kHz) */
  {
    const osThreadAttr_t attr = {
      .name = "IMU_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityAboveNormal3,
    };
    taskHandle_IMU = osThreadNew(Task_IMU, NULL, &attr);
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

  /* Motor control task (100Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "Motor_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityAboveNormal1,
    };
    taskHandle_Motor = osThreadNew(Task_Motor, NULL, &attr);
  }

  /* Remote control task (100Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "Remote_Task",
      .stack_size = 2048,
      .priority = (osPriority_t) osPriorityNormal,
    };
    taskHandle_Remote = osThreadNew(Task_Remote, NULL, &attr);
  }

  /* Monitor task (10Hz) */
  {
    const osThreadAttr_t attr = {
      .name = "Monitor_Task",
      .stack_size = 1024,
      .priority = (osPriority_t) osPriorityBelowNormal,
    };
    taskHandle_Monitor = osThreadNew(Task_Monitor, NULL, &attr);
  }

  /* Debug task (1Hz) */
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

/* ============================================================================
 *                          TASK IMPLEMENTATIONS
 * ============================================================================ */

/**
  * @brief IMU data acquisition task (highest priority, 1kHz)
  */
void Task_IMU(void *argument)
{
    (void)argument;

    if (!ICM42688_Init()) {
        g_system_status |= 0x01;
        vTaskSuspend(NULL);
    }

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        ICM42688_Update();
        g_imu_update_count++;
        osDelayUntil(tick + 1);
        tick += 1;
    }
}

/**
  * @brief Balance control task (500Hz)
  */
void Task_Balance(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* TODO: Implement balance control algorithm */
        /* Use g_imu_accel_x_filtered, g_imu_gyro_x_filtered, etc. */

        osDelayUntil(tick + 2);
        tick += 2;
    }
}

/**
  * @brief Motor control task (100Hz)
  */
void Task_Motor(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        /* TODO: Implement motor control via CAN */
        /* Use osMutexAcquire/Release to protect CAN bus */

        g_motor_update_count++;
        osDelayUntil(tick + 10);
        tick += 10;
    }
}

/**
  * @brief Remote control task (100Hz)
  */
void Task_Remote(void *argument)
{
    (void)argument;

    NRF24L01_RX_Init();
    if (!NRF24L01_RX_WaitForPairing()) {
        g_system_status |= 0x02;
    }

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        if (NRF24L01_RX_ReadData()) {
            g_remote_update_count++;
        }
        osDelayUntil(tick + 10);
        tick += 10;
    }
}

/**
  * @brief System monitoring task (10Hz)
  */
void Task_Monitor(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        if (!NRF24L01_RX_IsOnline()) {
            g_system_status |= 0x04;
        } else {
            g_system_status &= ~0x04;
        }

        osDelayUntil(tick + 100);
        tick += 100;
    }
}

/**
  * @brief Debug output task (1Hz)
  */
void Task_Debug(void *argument)
{
    (void)argument;

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        osDelayUntil(tick + 1000);
        tick += 1000;
    }
}

/* USER CODE END Application */

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

