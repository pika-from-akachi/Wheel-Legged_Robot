/**
  ******************************************************************************
  * @file    nrf24l01_example.c
  * @brief   NRF24L01+ Usage Examples for Wheel-Legged Robot
  * @note    Demonstrates TX and RX modes for remote control
  *
  *          Hardware Connection (STM32F407):
  *          NRF24L01 Module    STM32F407
  *          ─────────────────────────────
  *          VCC         ───►   3.3V (⚠️ NOT 5V!)
  *          GND         ───►   GND
  *          CE          ───►   PC8
  *          CSN         ───►   PC9
  *          SCK         ───►   PC10 (SPI3_SCK)
  *          MOSI        ───►   PC12 (SPI3_MOSI)
  *          MISO        ───►   PC11 (SPI3_MISO)
  *          IRQ         ───►   PC7 (optional)
  *
  * @author  Mingyue Class 2026
  * @date    2026-04-25
  ******************************************************************************
  */

#include "nrf24l01.h"
#include "nrf24l01_hal_config.h"
#include "main.h"
#include <string.h>

/* ============================================================================
 *                          GLOBAL VARIABLES
 * ============================================================================ */

// NRF24L01 handle
NRF24L01_Handle_t nrf_handle;

/* ============================================================================
 *                          EXAMPLE 1: TRANSMITTER (REMOTE CONTROLLER)
 * ============================================================================ */

/**
 * @brief Initialize NRF24L01 as transmitter
 */
void NRF24L01_TX_Example_Init(void)
{
    // Initialize GPIO pins for NRF24L01
    NRF24L01_GPIO_Init();

    // Configure NRF24L01
    NRF24L01_Init_t config = {
        .hspi = &hspi3,                     // SPI3 handle
        .ce_port = NRF24L01_CE_PORT,        // PC8
        .ce_pin = NRF24L01_CE_PIN,
        .csn_port = NRF24L01_CSN_PORT,      // PC9
        .csn_pin = NRF24L01_CSN_PIN,
        .irq_port = NRF24L01_IRQ_PORT,      // PC7
        .irq_pin = NRF24L01_IRQ_PIN,
        .channel = 70,                      // RF channel 70 (2.470GHz)
        .payload_width = 16,                // 16 bytes payload
        .data_rate = NRF24L01_RF_DR_2MBPS,  // 2Mbps data rate
        .tx_power = NRF24L01_RF_PWR_0DBM,   // 0dBm TX power
        .address = {0x34, 0x43, 0x10, 0x10, 0x01}, // TX/RX address
        .auto_ack = true,                   // Enable auto-ack
        .dynamic_payload = false            // Fixed payload width
    };

    // Initialize NRF24L01
    if (NRF24L01_Init(&nrf_handle, &config) != HAL_OK) {
        Error_Handler();
    }

    // Set to TX mode
    NRF24L01_SetTxMode(&nrf_handle);
}

/**
 * @brief Send remote control data
 * @param left_speed: Left motor speed (-100 to 100)
 * @param right_speed: Right motor speed (-100 to 100)
 * @param button: Button state (bitmask)
 */
void NRF24L01_SendControlData(int8_t left_speed, int8_t right_speed, uint8_t button)
{
    uint8_t tx_buffer[16];

    // Pack control data
    tx_buffer[0] = 0xAA;               // Header byte
    tx_buffer[1] = (uint8_t)left_speed;
    tx_buffer[2] = (uint8_t)right_speed;
    tx_buffer[3] = button;
    tx_buffer[4] = 0x55;               // Tail byte

    // Calculate checksum (simple XOR)
    tx_buffer[5] = tx_buffer[0] ^ tx_buffer[1] ^ tx_buffer[2] ^ tx_buffer[3] ^ tx_buffer[4];

    // Transmit data
    NRF24L01_Transmit(&nrf_handle, tx_buffer, 16);

    // Wait for transmission complete
    while (NRF24L01_IsSending(&nrf_handle)) {
        HAL_Delay(1);
    }

    // Check transmission status
    NRF24L01_Status_t status = NRF24L01_GetStatus(&nrf_handle);

    if (status.tx_ds) {
        // Transmission successful
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Toggle LED (adjust pin as needed)
    } else if (status.max_rt) {
        // Max retransmit reached - no ACK received
        NRF24L01_FlushTx(&nrf_handle);
    }

    // Clear status flags
    NRF24L01_WriteRegister(&nrf_handle, NRF24L01_REG_STATUS,
                          NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);
}

/**
 * @brief Main loop for transmitter
 */
void NRF24L01_TX_Example_Loop(void)
{
    int8_t left_speed = 50;   // Example: 50% speed forward
    int8_t right_speed = 50;
    uint8_t button = 0x01;    // Button 1 pressed

    while (1) {
        NRF24L01_SendControlData(left_speed, right_speed, button);
        HAL_Delay(20); // 50Hz update rate
    }
}

/* ============================================================================
 *                          EXAMPLE 2: RECEIVER (ROBOT)
 * ============================================================================ */

/**
 * @brief Initialize NRF24L01 as receiver
 */
void NRF24L01_RX_Example_Init(void)
{
    // Initialize GPIO pins for NRF24L01
    NRF24L01_GPIO_Init();

    // Configure NRF24L01
    NRF24L01_Init_t config = {
        .hspi = &hspi3,                     // SPI3 handle
        .ce_port = NRF24L01_CE_PORT,        // PC8
        .ce_pin = NRF24L01_CE_PIN,
        .csn_port = NRF24L01_CSN_PORT,      // PC9
        .csn_pin = NRF24L01_CSN_PIN,
        .irq_port = NRF24L01_IRQ_PORT,      // PC7
        .irq_pin = NRF24L01_IRQ_PIN,
        .channel = 70,                      // Same channel as transmitter
        .payload_width = 16,                // Same payload width
        .data_rate = NRF24L01_RF_DR_2MBPS,  // Same data rate
        .tx_power = NRF24L01_RF_PWR_0DBM,
        .address = {0x34, 0x43, 0x10, 0x10, 0x01}, // Same address
        .auto_ack = true,
        .dynamic_payload = false
    };

    // Initialize NRF24L01
    if (NRF24L01_Init(&nrf_handle, &config) != HAL_OK) {
        Error_Handler();
    }

    // Set to RX mode
    NRF24L01_SetRxMode(&nrf_handle);
}

/**
 * @brief Receive and process control data
 * @param left_speed: Pointer to store left motor speed
 * @param right_speed: Pointer to store right motor speed
 * @param button: Pointer to store button state
 * @retval 1 if data received, 0 otherwise
 */
uint8_t NRF24L01_ReceiveControlData(int8_t *left_speed, int8_t *right_speed, uint8_t *button)
{
    uint8_t rx_buffer[16];
    uint8_t length;

    // Check if data is ready
    if (!NRF24L01_DataReady(&nrf_handle)) {
        return 0;
    }

    // Receive data
    if (NRF24L01_Receive(&nrf_handle, rx_buffer, &length) != HAL_OK) {
        return 0;
    }

    // Validate header and tail
    if (rx_buffer[0] != 0xAA || rx_buffer[4] != 0x55) {
        return 0;
    }

    // Validate checksum
    uint8_t checksum = rx_buffer[0] ^ rx_buffer[1] ^ rx_buffer[2] ^ rx_buffer[3] ^ rx_buffer[4];
    if (checksum != rx_buffer[5]) {
        return 0;
    }

    // Extract control data
    *left_speed = (int8_t)rx_buffer[1];
    *right_speed = (int8_t)rx_buffer[2];
    *button = rx_buffer[3];

    return 1;
}

/**
 * @brief Main loop for receiver
 */
void NRF24L01_RX_Example_Loop(void)
{
    int8_t left_speed, right_speed;
    uint8_t button;

    while (1) {
        if (NRF24L01_ReceiveControlData(&left_speed, &right_speed, &button)) {
            // Apply motor control
            // Example: Use EL05 or M0601C motor drivers
            // EL05_VelocityControl(&motor_left, left_speed * 0.1f, 5.0f);
            // EL05_VelocityControl(&motor_right, right_speed * 0.1f, 5.0f);

            // Toggle LED to indicate data received (adjust pin as needed)
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        }

        HAL_Delay(10); // 100Hz check rate
    }
}

/* ============================================================================
 *                          EXAMPLE 3: BIDIRECTIONAL COMMUNICATION
 * ============================================================================ */

/**
 * @brief Switch between TX and RX mode
 * @note  For bidirectional communication, you need to switch modes
 */
void NRF24L01_Bidirectional_Example(void)
{
    uint32_t last_tx_time = 0;
    uint8_t tx_buffer[16] = {0xAA, 0x01, 0x02, 0x03, 0x55, 0xFF};
    uint8_t rx_buffer[16];
    uint8_t rx_length;

    while (1) {
        uint32_t current_time = HAL_GetTick();

        // Transmit every 50ms
        if (current_time - last_tx_time >= 50) {
            last_tx_time = current_time;

            // Switch to TX mode
            NRF24L01_SetTxMode(&nrf_handle);

            // Transmit data
            NRF24L01_Transmit(&nrf_handle, tx_buffer, 16);

            // Wait for transmission complete
            while (NRF24L01_IsSending(&nrf_handle)) {
                HAL_Delay(1);
            }

            // Clear status
            NRF24L01_Status_t status = NRF24L01_GetStatus(&nrf_handle);
            NRF24L01_WriteRegister(&nrf_handle, NRF24L01_REG_STATUS,
                                  NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);

            // Switch back to RX mode
            NRF24L01_SetRxMode(&nrf_handle);
        }

        // Check for received data
        if (NRF24L01_DataReady(&nrf_handle)) {
            NRF24L01_Receive(&nrf_handle, rx_buffer, &rx_length);
            // Process received data...
        }

        HAL_Delay(1);
    }
}

/* ============================================================================
 *                          EXAMPLE 4: INTERRUPT-DRIVEN RX
 * ============================================================================ */

/**
 * @brief External interrupt callback for IRQ pin (PC7)
 * @param GPIO_Pin: Pin that triggered interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == NRF24L01_IRQ_PIN) {
        NRF24L01_Status_t status = NRF24L01_GetStatus(&nrf_handle);

        if (status.rx_dr) {
            // Data received - process in main loop
            // Set a flag to indicate data ready
        }

        if (status.tx_ds) {
            // Transmission successful
        }

        if (status.max_rt) {
            // Max retransmit reached
            NRF24L01_FlushTx(&nrf_handle);
        }

        // Clear interrupts
        NRF24L01_WriteRegister(&nrf_handle, NRF24L01_REG_STATUS,
                              NRF24L01_STATUS_RX_DR | NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);
    }
}

/* ============================================================================
 *                          INTEGRATION WITH MOTOR DRIVERS
 * ============================================================================ */

/**
 * @brief Example: Control EL05 motors with NRF24L01 remote
 */
void NRF24L01_EL05_Integration_Example(void)
{
    // Initialize EL05 motors (see EL05 documentation)
    // EL05_Init(&hcan1);
    // EL05_StartReception();

    // Initialize NRF24L01 as receiver
    NRF24L01_RX_Example_Init();

    int8_t left_speed, right_speed;
    uint8_t button;

    while (1) {
        if (NRF24L01_ReceiveControlData(&left_speed, &right_speed, &button)) {
            // Convert speed command to rad/s
            float left_vel = left_speed * 0.1f;  // Scale to rad/s
            float right_vel = right_speed * 0.1f;

            // Apply velocity control to EL05 motors
            // EL05_MotorHandle_t motor_left = {.can_id = 0x01};
            // EL05_MotorHandle_t motor_right = {.can_id = 0x02};

            // EL05_VelocityControl(&motor_left, left_vel, 5.0f);
            // EL05_VelocityControl(&motor_right, right_vel, 5.0f);

            // Handle button commands
            if (button & 0x01) {
                // Button 1: Enable motors
                // EL05_Enable(&motor_left);
                // EL05_Enable(&motor_right);
            }
            if (button & 0x02) {
                // Button 2: Disable motors
                // EL05_Disable(&motor_left);
                // EL05_Disable(&motor_right);
            }
        }

        HAL_Delay(10);
    }
}

/* ============================================================================
 *                          STM32CUBEMX CONFIGURATION
 * ============================================================================ */

/**
 * @brief STM32CubeMX Configuration Guide
 *
 * To configure SPI3 and GPIO pins in STM32CubeMX:
 *
 * 1. SPI3 Configuration:
 *    - Go to Connectivity -> SPI3
 *    - Mode: Full-Duplex Master
 *    - Hardware NSS Signal: Disable
 *    - Configuration -> Parameter Settings:
 *      * Frame Format: Motorola
 *      * Clock Polarity (CPOL): Low
 *      * Clock Phase (CPHA): 1 Edge
 *      * Data Size: 8 Bits
 *      * First Bit: MSB First
 *      * Prescaler: 16 (for 10.5MHz clock)
 *    - Pinout:
 *      * PC10: SPI3_SCK
 *      * PC11: SPI3_MISO
 *      * PC12: SPI3_MOSI
 *
 * 2. GPIO Configuration:
 *    - PC8: GPIO_Output (CE)
 *      * GPIO mode: Output Push Pull
 *      * GPIO Pull-up/Pull-down: No pull-up and no pull-down
 *      * Maximum output speed: High
 *      * User Label: NRF_CE
 *
 *    - PC9: GPIO_Output (CSN)
 *      * GPIO mode: Output Push Pull
 *      * GPIO Pull-up/Pull-down: No pull-up and no pull-down
 *      * Maximum output speed: High
 *      * User Label: NRF_CSN
 *
 *    - PC7: GPIO_EXTI7 (IRQ, optional)
 *      * GPIO mode: External Interrupt Mode with Falling edge trigger detection
 *      * GPIO Pull-up/Pull-down: Pull-up
 *      * User Label: NRF_IRQ
 *
 * 3. NVIC Configuration (if using IRQ):
 *    - Go to System Core -> NVIC
 *    - Enable EXTI line[9:5] interrupts
 *    - Set appropriate priority
 *
 * 4. Clock Configuration:
 *    - APB1 Peripheral Clocks: Enable SPI3
 *    - SPI3 maximum clock: 10.5MHz (PCLK1 / 16)
 */
