/**
  ******************************************************************************
  * @file    nrf24l01_rx.h
  * @brief   NRF24L01+ Receiver Driver for Wheel-Legged Robot
  * @note    Simplified driver for receiver only
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

#ifndef __NRF24L01_RX_H
#define __NRF24L01_RX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *                          HARDWARE PIN DEFINITIONS
 * ============================================================================ */

#define NRF24L01_SPI                    hspi3
extern SPI_HandleTypeDef hspi3;

#define NRF24L01_CE_PORT                GPIOC
#define NRF24L01_CE_PIN                 GPIO_PIN_8

#define NRF24L01_CSN_PORT               GPIOC
#define NRF24L01_CSN_PIN                GPIO_PIN_9

#define NRF24L01_IRQ_PORT               GPIOC
#define NRF24L01_IRQ_PIN                GPIO_PIN_7

/* ============================================================================
 *                          NRF24L01 CONFIGURATION
 * ============================================================================ */

#define NRF24L01_SPI_TIMEOUT            100
#define NRF24L01_MAX_PAYLOAD_WIDTH      32
#define NRF24L01_ADDR_WIDTH             5

/* NRF24L01 Commands */
#define NRF24L01_CMD_READ_REG           0x00
#define NRF24L01_CMD_WRITE_REG          0x20
#define NRF24L01_CMD_RD_RX_PAYLOAD      0x61
#define NRF24L01_CMD_FLUSH_RX           0xE2
#define NRF24L01_CMD_NOP                0xFF

/* NRF24L01 Registers */
#define NRF24L01_REG_CONFIG             0x00
#define NRF24L01_REG_EN_AA              0x01
#define NRF24L01_REG_EN_RXADDR          0x02
#define NRF24L01_REG_SETUP_AW           0x03
#define NRF24L01_REG_RF_CH              0x05
#define NRF24L01_REG_RF_SETUP           0x06
#define NRF24L01_REG_STATUS             0x07
#define NRF24L01_REG_RX_ADDR_P0         0x0A
#define NRF24L01_REG_TX_ADDR            0x10
#define NRF24L01_REG_RX_PW_P0           0x11
#define NRF24L01_REG_FIFO_STATUS        0x17

/* STATUS Register Bits */
#define NRF24L01_STATUS_RX_DR           (1 << 6)
#define NRF24L01_STATUS_TX_DS           (1 << 5)
#define NRF24L01_STATUS_MAX_RT          (1 << 4)

/* RF Settings */
#define NRF24L01_RF_DR_250KBPS        0x20
#define NRF24L01_RF_DR_1MBPS          0x00
#define NRF24L01_RF_DR_2MBPS          0x08
#define NRF24L01_RF_PWR_0DBM          0x06

/* ============================================================================
 *                          DATA STRUCTURES
 * ============================================================================ */

/**
 * @brief Remote control data structure (matches protocol from datasheet)
 */
typedef struct {
    uint8_t right_joystick_x;   // Right joystick X (128=center, <128=left, >128=right)
    uint8_t right_joystick_y;   // Right joystick Y (128=center, <128=forward, >128=backward)
    uint8_t left_joystick_x;    // Left joystick X (128=center, <128=left, >128=right)
    uint8_t left_joystick_y;    // Left joystick Y (128=center, <128=forward, >128=backward)
    uint8_t button_state;       // Button state (bitmask: bit0=KEY1, bit1=KEY2, bit2=KEY3, bit3=KEY4, bit4=LB, bit5=RB)
    uint8_t rolling_code;       // Rolling code from remote
    uint8_t data_valid;         // Data validation flag
    uint32_t timestamp;         // Reception timestamp (ms)
} RemoteControlData_t;

/**
 * @brief NRF24L01 receiver handle
 */
typedef struct {
    SPI_HandleTypeDef *hspi;
    uint8_t payload_width;
    uint8_t rx_buffer[32];
    RemoteControlData_t rc_data;
    uint32_t last_receive_time;
    uint8_t is_initialized;
    uint8_t paired_address[5];      // Paired address after code matching
    uint8_t is_paired;              // Pairing status flag
    uint8_t rolling_code;           // Last received rolling code
} NRF24L01_RX_Handle_t;

/* ============================================================================
 *                          FUNCTION PROTOTYPES
 * ============================================================================ */

/* Initialization */
void NRF24L01_RX_Init(void);
void NRF24L01_RX_GPIO_Init(void);

/* Pairing */
uint8_t NRF24L01_RX_WaitForPairing(void);
uint8_t NRF24L01_RX_IsPaired(void);

/* Data Reception */
uint8_t NRF24L01_RX_CheckData(void);
uint8_t NRF24L01_RX_ReadData(void);
RemoteControlData_t* NRF24L01_RX_GetData(void);

/* Status Check */
uint8_t NRF24L01_RX_IsOnline(void);
uint32_t NRF24L01_RX_GetLastReceiveTime(void);

/* Low-Level Functions */
uint8_t NRF24L01_RX_ReadRegister(uint8_t reg);
void NRF24L01_RX_WriteRegister(uint8_t reg, uint8_t value);
void NRF24L01_RX_FlushRx(void);
void NRF24L01_RX_SetAddress(uint8_t *address);

/* Pin Control */
#define NRF24L01_CE_HIGH()      HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_SET)
#define NRF24L01_CE_LOW()       HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_RESET)
#define NRF24L01_CSN_HIGH()     HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_SET)
#define NRF24L01_CSN_LOW()      HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_RESET)

#ifdef __cplusplus
}
#endif

#endif /* __NRF24L01_RX_H */
