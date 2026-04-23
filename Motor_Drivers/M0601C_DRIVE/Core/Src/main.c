/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_driver.h" /* M0601C 电机驱动模块头文件 */
#include <string.h>
#include <stdio.h>
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

/* USER CODE BEGIN PV */
/* 全局变量：电机反馈数据（调试器可查看） */
MotorFeedback_t g_motorFeedback = {0};
MotorStatus_t g_motorStatus = {0};  /* 解析后的电机状态 */
uint8_t g_feedbackCount = 0; /* 收到的反馈帧计数 */

/* 全局调试信息实例 */
DebugInfo_t g_debug = {0};

/* 为了兼容性保留旧变量 */
uint8_t g_dmaRxStarted = 0;
uint8_t g_rxCpltCallback = 0;
uint8_t g_crcError = 0;
uint8_t g_rxBufRaw[10] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* 声明电机驱动函数 */
extern HAL_StatusTypeDef MOTOR_SendFeedbackCmd(uint8_t motorId);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /*
   * M0601C 电机持续运行程序
   * 电机ID: 1
   * 目标: 以最高转速 330 RPM 持续运行，并通过调试器查看反馈数据
   */

  /* 上电延时等待电机初始化 */
  HAL_Delay(2000);

  /* LED闪烁2次，指示程序启动 */
  for (int i = 0; i < 2; i++)
  {
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(200);
  }

  /* 启动 DMA 接收 */
  HAL_StatusTypeDef status = MOTOR_StartReceive();
  g_dmaRxStarted = (status == HAL_OK) ? 1 : 0;
  g_debug.dmaRxStarted = g_dmaRxStarted;

  /* 以 10 RPM 正转运行 */
  MOTOR_SetSpeed(1, 10);

  /* LED常亮指示电机运行中 */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 重新启动 DMA 接收（确保每次都能捕获新数据） */
    MOTOR_StartReceive();

    /* 发送反馈请求 */
    HAL_StatusTypeDef txStatus = MOTOR_SendFeedbackCmd(1);
    g_debug.txStatus = (txStatus == HAL_OK) ? 1 : 0;

    /* 等待电机返回数据 */
    HAL_Delay(100);

    /* 获取反馈数据 */
    MotorFeedback_t *pFeedback = MOTOR_GetFeedback();
    if (pFeedback->isValid)
    {
      /* 复制到全局变量供调试器查看 */
      memcpy(&g_motorFeedback, pFeedback, sizeof(MotorFeedback_t));

      /* 解析原始数据成物理量 */
      MOTOR_ParseFeedback(pFeedback, &g_motorStatus);

      g_feedbackCount++;

      /* 同步更新调试结构体 */
      g_debug.rxBufValid = 1;

      /* LED闪烁指示收到反馈 */
      HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
    }

    /* 更新调试结构体的计数器 */
    g_debug.rxCpltCallback = g_rxCpltCallback;
    g_debug.crcError = g_crcError;

    HAL_Delay(400);  /* 总共500ms */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
