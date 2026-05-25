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
#include "cmsis_os.h"
#include "can.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24l01_rx.h"
#include "icm42688.h"
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

// IMU数据（Keil Watch窗口查看）
extern volatile float g_imu_accel_x_g;
extern volatile float g_imu_accel_y_g;
extern volatile float g_imu_accel_z_g;
extern volatile float g_imu_gyro_x_dps;
extern volatile float g_imu_gyro_y_dps;
extern volatile float g_imu_gyro_z_dps;
extern volatile float g_imu_temperature_c;
extern volatile int16_t g_imu_accel_x_raw;
extern volatile int16_t g_imu_accel_y_raw;
extern volatile int16_t g_imu_accel_z_raw;
extern volatile int16_t g_imu_gyro_x_raw;
extern volatile int16_t g_imu_gyro_y_raw;
extern volatile int16_t g_imu_gyro_z_raw;
extern volatile uint8_t g_imu_who_am_i;
extern volatile uint8_t g_imu_init_status;
volatile uint8_t imu_initialized = 0;

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

// EL05电机句柄
EL05_MotorHandle_t motor1;

// M0601C电机UART句柄（由MX_USART1_UART_Init初始化）
UART_HandleTypeDef huart1;

// M0601C电机驱动调试变量（供motor_driver.c链接）
typedef struct {
    uint8_t rxCpltCallback;
    uint8_t crcError;
    uint8_t rxBufRaw[10];
    uint8_t rxBufValid;
    uint8_t lastCrcCalc;
    uint8_t lastCrcRecv;
} DebugInfo_t;
uint8_t g_rxCpltCallback = 0;
uint8_t g_crcError = 0;
uint8_t g_rxBufRaw[10] = {0};
DebugInfo_t g_debug = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* FreeRTOS tasks will handle all initialization and control */
  /* Do not initialize peripherals here - tasks will do it */

  /* 初始化EL05电机驱动 */
  EL05_Init(&hcan1);
  motor1.can_id = 0x7F;
  motor1.mode = EL05_MODE_MIT;
  motor1.state = EL05_STATE_DISABLE;
  motor1.is_online = 0;

  /* 设置挂载在总线的电机ID为2（参照例程SampleProgram的Set_CAN_ID方式） */
  if (EL05_SetMotorId(&motor1, 4) == HAL_OK) {
      HAL_Delay(50);      /* 等待CAN帧发送完成 */
      motor1.can_id = 4;  /* 更新本地句柄以匹配电机新ID */
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* This code will never execute - FreeRTOS tasks run independently */
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
/* Stub HAL_UART_Transmit — M0601C motor driver not used in this test */
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                     const uint8_t *pData, uint16_t Size,
                                     uint32_t Timeout)
{
    (void)huart; (void)pData; (void)Size; (void)Timeout;
    return HAL_OK;
}

/* Stub HAL_UART_Receive_DMA — M0601C motor driver not used in this test */
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart,
                                        uint8_t *pData, uint16_t Size)
{
    (void)huart; (void)pData; (void)Size;
    return HAL_OK;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
