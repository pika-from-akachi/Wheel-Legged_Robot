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
#include <string.h>
#include "nrf24l01_rx.h"
#include "icm42688.h"
#include "el05_motor.h"
#include "motor_driver.h"
//#include "m0601c_motor.h"   // disabled — not in build
//#include "lqr_control.h"
//#include "robot_model.h"
//#include "esp32_com.h"      // disabled — not in build
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

// EL05电机句柄 (4个关节电机)
EL05_MotorHandle_t g_el05_motors[4];

// UART句柄 (保留定义, 供Init函数引用)
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;   /* USART2: M0601C RS485 */
UART_HandleTypeDef huart3;

// EL05电机句柄 (单个, 供el05_motor.o的CAN RX回调使用)
EL05_MotorHandle_t motor1;

// ===== 调试变量 (motor_driver.c 引用) =====
uint8_t g_rxCpltCallback = 0;
uint8_t g_rxBufRaw[10] = {0};
uint8_t g_crcError = 0;

DebugInfo_t g_debug = {0};

// 轮毂电机ID扫描结果 (Debugger Watch用)
volatile uint8_t g_found_motor_id = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void MX_USART1_UART_Init(void);
HAL_StatusTypeDef MX_USART2_UART_Init(void);

/* ============================================================================
 *                          USART2 INIT (M0601C RS485)
 * ============================================================================
 * PD5 = USART2_RX, PD6 = USART2_TX
 * PD3 = RE# (active low), PD4 = DE (active high) — THVD1410DR
 * Baud rate: 115200, 8N1
 * ============================================================================ */

HAL_StatusTypeDef MX_USART2_UART_Init(void)
{
    HAL_StatusTypeDef status;
    /* Enable clocks */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Configure USART2 pins: PD5=RX, PD6=TX */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* Configure RS485 direction pins: PD3=RE#, PD4=DE */
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0;
    HAL_GPIO_Init(GPIOD, &gpio);
    /* THVD1410DR: RE#=0, DE=0 → 接收模式 (默认) */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

    /* UART configuration */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Enable USART2 interrupt for RS485 byte reception */
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

    return HAL_OK;
}

void MX_USART3_UART_Init(void);
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
  /* Initialize all motor drivers and communication peripherals */

  /* ===== EL05 Joint Motors (CAN, IDs 1-4) ===== */
  EL05_Init(&hcan1);

  uint8_t el05_ids[4] = {1, 2, 3, 4};
  for (int i = 0; i < 4; i++) {
      g_el05_motors[i].can_id = el05_ids[i];
      g_el05_motors[i].mode = EL05_MODE_MIT;
      g_el05_motors[i].state = EL05_STATE_DISABLE;
      g_el05_motors[i].is_online = 0;
  }

  /* 本地匹配, 不涉及CAN通信: 让CAN RX回调能识别电机1的响应帧 */
  motor1.can_id = 1;

  /* motor1.can_id = 1 : 让CAN RX回调能识别电机1的反馈帧 (不是改电机硬件ID) */
  motor1.can_id = 1;

  /* ===== M0601C Hub Motors (RS485 via USART2) ===== */
  extern volatile uint8_t debug_uart2_init_ok;
  debug_uart2_init_ok = (MX_USART2_UART_Init() == HAL_OK) ? 1 : 2;
  MOTOR_StartReceive();

  /* ===== ESP32 Communication (USART3) — DISABLED ===== */
  //MX_USART3_UART_Init();
  //ESP32_COM_Init(&huart3);
  //ESP32_COM_StartRx();

  // ===== Robot Model & LQR disabled for motor test =====
  //ROBOT_Init();
  //LQR_Init(&g_lqr_controller);
  //LQR_SetDefaultTuning(&g_lqr_tuning);
  //LQR_ComputeGains(&g_lqr_tuning, &g_lqr_controller.gain);
  //LQR_SetMode(&g_lqr_controller, ROBOT_MODE_STANDING);

  /* USER CODE END 2 */

  /* ===== 轮毂电机 ID=1 驱动测试 (跟Task_EL05_Motor一致) ===== */
  MOTOR_SendModeSwitchCmd(1, MOTOR_CTRL_SPEED);
  HAL_Delay(20);
  MOTOR_SetSpeed(1, 50);

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

/* ============================================================================
 *                          USART1 INIT (M0601C RS485)
 * ============================================================================
 * PB6 = USART1_TX, PB7 = USART1_RX
 * PE0 = RS485 Direction Control (DE/RE)
 * Baud rate: 115200, 8N1
 * ============================================================================ */

void MX_USART1_UART_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* Configure USART1 pins: PB6=TX, PB7=RX */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Configure RS485 direction pin: PE0 */
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0;
    HAL_GPIO_Init(GPIOE, &gpio);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); /* Default RX */

    /* UART configuration */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }

    /* Enable USART1 interrupt for RS485 byte reception */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

/* ============================================================================
 *                          USART3 INIT (ESP32 COMMUNICATION)
 * ============================================================================
 * PB10 = USART3_TX, PB11 = USART3_RX
 * Baud rate: 921600 (high speed for real-time control data)
 * ============================================================================ */

void MX_USART3_UART_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure USART3 pins: PB10=TX, PB11=RX */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* UART configuration */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 921600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }

    /* Enable USART3 interrupt */
    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

/* ============================================================================
 *                          UART INTERRUPT HANDLERS
 * ============================================================================ */

/**
 * @brief USART1 IRQ handler — DISABLED (not in build)
 */
void USART1_IRQHandler(void)
{
    /* UART not initialized — should never fire */
}

/**
 * @brief USART2 IRQ handler
 * @note  Forward to HAL for motor RS485 reception (IT mode)
 */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief USART3 IRQ handler — DISABLED (not in build)
 */
void USART3_IRQHandler(void)
{
    /* UART not initialized — should never fire */
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
