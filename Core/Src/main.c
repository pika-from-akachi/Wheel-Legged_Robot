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
#include "can.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

/* USER CODE BEGIN PV */
// 遥控器数据（Keil Watch窗口查看）
RemoteControlData_t remote_data = {0};
volatile int16_t debug_right_x = 0;
volatile int16_t debug_right_y = 0;
volatile int16_t debug_left_x = 0;
volatile int16_t debug_left_y = 0;
volatile uint8_t debug_buttons = 0;
volatile uint8_t pairing_status = 0;
volatile uint8_t online_status = 0;
volatile uint32_t packet_count = 0;
volatile uint8_t ack_payload_sent = 0;
volatile uint8_t fallback_tx_used = 0;

// Extern debug variables from nrf24l01_rx.c
extern volatile uint8_t g_ack_payload_sent;
extern volatile uint8_t g_fallback_tx_used;
extern volatile uint8_t g_debug_en_aa;
extern volatile uint8_t g_debug_en_rxaddr;
extern volatile uint8_t g_debug_rx_pw_p0;
extern volatile uint8_t g_debug_feature;
extern volatile uint8_t g_debug_dynpd;
extern volatile uint8_t g_debug_config;
extern volatile uint8_t g_debug_status;
extern volatile uint8_t g_debug_fifo_status;
extern volatile uint8_t g_debug_rx_dr_count;
extern volatile uint8_t g_debug_ce_state;
extern volatile uint8_t g_debug_rx_addr[5];
extern volatile uint8_t g_debug_tx_addr[5];
extern volatile uint8_t g_debug_first_packet[4];
extern volatile uint8_t g_debug_packets_on_new_addr;
extern volatile uint8_t g_debug_last_rolling_code;
extern volatile uint8_t g_debug_observe_tx;
extern volatile uint8_t g_debug_arc_cnt;
extern volatile uint8_t g_debug_rf_setup;
extern volatile uint8_t g_debug_rf_ch;
extern volatile uint8_t g_debug_config_after_switch;
extern volatile uint8_t g_debug_ce_after_switch;
extern volatile uint8_t g_debug_new_address_sent[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  // 初始化NRF24L01接收器
  NRF24L01_RX_Init();

  // 等待对码（10秒超时）
  if (!NRF24L01_RX_WaitForPairing()) {
    pairing_status = 0;  // 对码失败
  } else {
    pairing_status = 1;  // 对码成功
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 读取遥控器数据
    if (NRF24L01_RX_ReadData()) {
      RemoteControlData_t *rc = NRF24L01_RX_GetData();
      remote_data = *rc;

      // 转换摇杆值到中心范围 (-128~127)
      debug_right_x = (int16_t)rc->right_joystick_x - 128;
      debug_right_y = (int16_t)rc->right_joystick_y - 128;
      debug_left_x = (int16_t)rc->left_joystick_x - 128;
      debug_left_y = (int16_t)rc->left_joystick_y - 128;
      debug_buttons = rc->button_state;

      packet_count++;

      // 在这里添加你的控制逻辑
      // 例如：电机控制、按键处理等
    }

    // 检查在线状态
    online_status = NRF24L01_RX_IsOnline();

    // 同步调试变量
    ack_payload_sent = g_ack_payload_sent;
    fallback_tx_used = g_fallback_tx_used;

    // 安全保护：遥控器离线时停止电机
    if (!online_status) {
      // 停止电机
    }

    HAL_Delay(10);  // 100Hz更新率
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
