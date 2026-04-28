/**
  ******************************************************************************
  * @file    main.c
  * @brief   NRF遥控器测试程序 - Keil Debugger实时监控
  * @note    在Keil的Watch窗口添加以下变量查看：
  *          - remote_data (结构体)
  *          - debug_right_x, debug_right_y, debug_left_x, debug_left_y
  *          - debug_buttons
  *          - pairing_status
  *          - online_status
  *
  * @author  Mingyue Class 2026
  * @date    2026-04-25
  ******************************************************************************
  */

#include "main.h"
#include "nrf24l01_rx.h"
#include <stdio.h>

/* ============================================================================
 *                          GLOBAL VARIABLES FOR DEBUG
 * ============================================================================ */

// 遥控器数据结构体（在Watch窗口查看此结构体）
RemoteControlData_t remote_data = {0};

// 调试变量：转换后的摇杆值（-128 ~ 127）
volatile int16_t debug_right_x = 0;     // 右摇杆X
volatile int16_t debug_right_y = 0;     // 右摇杆Y
volatile int16_t debug_left_x = 0;      // 左摇杆X
volatile int16_t debug_left_y = 0;      // 左摇杆Y

// 调试变量：按键状态
volatile uint8_t debug_buttons = 0;     // 按键状态位掩码
volatile uint8_t debug_key1 = 0;        // KEY1状态 (0=松开, 1=按下)
volatile uint8_t debug_key2 = 0;        // KEY2状态
volatile uint8_t debug_key3 = 0;        // KEY3状态
volatile uint8_t debug_key4 = 0;        // KEY4状态
volatile uint8_t debug_lb = 0;          // LB状态
volatile uint8_t debug_rb = 0;          // RB状态

// 调试变量：系统状态
volatile uint8_t pairing_status = 0;    // 对码状态 (0=未对码, 1=已对码)
volatile uint8_t online_status = 0;     // 在线状态 (0=离线, 1=在线)
volatile uint32_t last_receive_time = 0; // 最后接收时间
volatile uint32_t packet_count = 0;     // 接收数据包计数
volatile uint8_t rolling_code = 0;      // 滚动码

// 调试变量：原始数据
volatile uint8_t raw_byte0 = 0;         // 原始数据字节0 (应为0x01)
volatile uint8_t raw_byte1 = 0;         // 原始数据字节1 (应为0x03)
volatile uint8_t raw_byte3 = 0;         // 原始数据字节3 (应为0x11)
volatile uint8_t raw_checksum = 0;      // 原始校验和
volatile uint8_t calc_checksum = 0;     // 计算的校验和

/* ============================================================================
 *                          FUNCTION PROTOTYPES
 * ============================================================================ */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void Update_Debug_Variables(void);
static void Process_Remote_Control(void);

/* ============================================================================
 *                          MAIN FUNCTION
 * ============================================================================ */

int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    // Reset of all peripherals, Initializes the Flash interface and the Systick
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_SPI3_Init();

    // Initialize NRF24L01 receiver
    NRF24L01_RX_Init();

    // Wait for pairing with remote controller (timeout 10 seconds)
    pairing_status = 0;
    if (NRF24L01_RX_WaitForPairing()) {
        pairing_status = 1;  // Pairing successful
    } else {
        pairing_status = 0;  // Pairing failed
        // LED indicator for pairing failed (if available)
        // Error_Handler();
    }

    /* Infinite loop */
    while (1)
    {
        // Read remote control data
        if (NRF24L01_RX_ReadData()) {
            // Get remote data
            RemoteControlData_t *rc = NRF24L01_RX_GetData();

            // Copy to global variable for debug
            remote_data = *rc;

            // Update debug variables
            Update_Debug_Variables();

            // Process remote control commands
            Process_Remote_Control();

            // Update packet counter
            packet_count++;
        }

        // Check online status (timeout 500ms)
        online_status = NRF24L01_RX_IsOnline();
        last_receive_time = NRF24L01_RX_GetLastReceiveTime();

        // Small delay to avoid overwhelming the system
        HAL_Delay(10);  // 100Hz update rate
    }
}

/* ============================================================================
 *                          DEBUG FUNCTIONS
 * ============================================================================ */

/**
 * @brief Update debug variables for Keil Watch window
 * @note  Add these variables to Watch window in Keil debugger
 */
static void Update_Debug_Variables(void)
{
    // Convert joystick values to centered range (-128 ~ 127)
    debug_right_x = (int16_t)remote_data.right_joystick_x - 128;
    debug_right_y = (int16_t)remote_data.right_joystick_y - 128;
    debug_left_x = (int16_t)remote_data.left_joystick_x - 128;
    debug_left_y = (int16_t)remote_data.left_joystick_y - 128;

    // Button state
    debug_buttons = remote_data.button_state;
    debug_key1 = (debug_buttons & 0x01) ? 1 : 0;  // Bit0: KEY1
    debug_key2 = (debug_buttons & 0x02) ? 1 : 0;  // Bit1: KEY2
    debug_key3 = (debug_buttons & 0x04) ? 1 : 0;  // Bit2: KEY3
    debug_key4 = (debug_buttons & 0x08) ? 1 : 0;  // Bit3: KEY4
    debug_lb = (debug_buttons & 0x10) ? 1 : 0;    // Bit4: LB
    debug_rb = (debug_buttons & 0x20) ? 1 : 0;    // Bit5: RB

    // Rolling code
    rolling_code = remote_data.rolling_code;

    // Raw data (for protocol verification)
    raw_byte0 = remote_data.right_joystick_x;  // Placeholder, need actual raw buffer
    // Note: To see raw bytes, you'd need to expose rx_buffer from nrf_rx_handle
}

/**
 * @brief Process remote control commands
 * @note  Add your motor control logic here
 */
static void Process_Remote_Control(void)
{
    // Example: Motor control based on joystick values
    // Convert joystick values to motor speeds
    int left_motor_speed = debug_left_y;   // Left joystick Y controls left motor
    int right_motor_speed = debug_right_y; // Right joystick Y controls right motor

    // Apply dead zone (±10)
    if (abs(left_motor_speed) < 10) {
        left_motor_speed = 0;
    }
    if (abs(right_motor_speed) < 10) {
        right_motor_speed = 0;
    }

    // Example: Control motors
    // EL05_VelocityControl(&motor_left, left_motor_speed * 0.1f, 5.0f);
    // EL05_VelocityControl(&motor_right, right_motor_speed * 0.1f, 5.0f);

    // Example: Handle button commands
    if (debug_key1) {
        // KEY1 pressed: Enable motors
        // EL05_Enable(&motor_left);
        // EL05_Enable(&motor_right);
    }

    if (debug_key2) {
        // KEY2 pressed: Disable motors
        // EL05_Disable(&motor_left);
        // EL05_Disable(&motor_right);
    }

    // Safety: Stop motors if remote is offline
    if (!online_status) {
        // EL05_Disable(&motor_left);
        // EL05_Disable(&motor_right);
    }
}

/* ============================================================================
 *                          PERIPHERAL INITIALIZATION
 * ============================================================================ */

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void)
{
    // This function should be generated by STM32CubeMX
    // Add your clock configuration here
}

/**
 * @brief GPIO Initialization
 */
static void MX_GPIO_Init(void)
{
    // This function should be generated by STM32CubeMX
    // GPIO initialization for your board
}

/**
 * @brief SPI3 Initialization
 */
static void MX_SPI3_Init(void)
{
    // This function should be generated by STM32CubeMX
    // Example SPI3 configuration:

    /*
    SPI_HandleTypeDef hspi3;

    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        Error_Handler();
    }
    */
}

/* ============================================================================
 *                          ERROR HANDLER
 * ============================================================================ */

/**
 * @brief This function is executed in case of error occurrence.
 */
void Error_Handler(void)
{
    // User can add his own implementation to report the HAL error return state
    __disable_irq();
    while (1) {
        // Stay here for debugger
    }
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    // User can add his own implementation to report the file name and line number
}
#endif
