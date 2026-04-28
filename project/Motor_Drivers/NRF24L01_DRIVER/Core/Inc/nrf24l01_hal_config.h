/**
  ******************************************************************************
  * @file    nrf24l01_hal_config.h
  * @brief   NRF24L01+ Hardware Abstraction Layer Configuration
  * @note    Hardware-specific pin definitions for STM32F407
  *
  *          Hardware Connection:
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

#ifndef __NRF24L01_HAL_CONFIG_H
#define __NRF24L01_HAL_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* ============================================================================
 *                          HARDWARE PIN DEFINITIONS
 * ============================================================================ */

/**
 * @brief NRF24L01 SPI Interface
 * @note  Using SPI3 on STM32F407
 */
#define NRF24L01_SPI                    hspi3
extern SPI_HandleTypeDef hspi3;

/**
 * @brief NRF24L01 CE Pin (Chip Enable)
 * @note  Controls TX/RX mode switching
 */
#define NRF24L01_CE_PORT                GPIOC
#define NRF24L01_CE_PIN                 GPIO_PIN_8

/**
 * @brief NRF24L01 CSN Pin (Chip Select Not)
 * @note  SPI chip select (active low)
 */
#define NRF24L01_CSN_PORT               GPIOC
#define NRF24L01_CSN_PIN                GPIO_PIN_9

/**
 * @brief NRF24L01 IRQ Pin (Interrupt Request)
 * @note  Active low interrupt (optional)
 */
#define NRF24L01_IRQ_PORT               GPIOC
#define NRF24L01_IRQ_PIN                GPIO_PIN_7

/* ============================================================================
 *                          GPIO INITIALIZATION
 * ============================================================================ */

/**
 * @brief Initialize NRF24L01 GPIO pins
 * @note  Call this function in main.c before NRF24L01_Init()
 */
static inline void NRF24L01_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIOC clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure CE Pin (PC8) - Output Push-Pull */
    GPIO_InitStruct.Pin = NRF24L01_CE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStruct);

    /* Configure CSN Pin (PC9) - Output Push-Pull */
    GPIO_InitStruct.Pin = NRF24L01_CSN_PIN;
    HAL_GPIO_Init(NRF24L01_CSN_PORT, &GPIO_InitStruct);

    /* Configure IRQ Pin (PC7) - Input with Interrupt (optional) */
    GPIO_InitStruct.Pin = NRF24L01_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // IRQ is active low
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NRF24L01_IRQ_PORT, &GPIO_InitStruct);

    /* Set initial states */
    HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_SET);

    /* Configure EXTI interrupt for IRQ pin (optional) */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
 * @brief Initialize SPI3 for NRF24L01
 * @note  Call this function in main.c or use STM32CubeMX to configure SPI3
 */
static inline void NRF24L01_SPI_Init(void)
{
    /* SPI3 initialization should be done in STM32CubeMX */
    /* If not using CubeMX, initialize SPI3 here:

    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 168MHz/16 = 10.5MHz
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
 *                          PIN CONTROL MACROS
 * ============================================================================ */

/**
 * @brief Set CE pin high
 */
#define NRF24L01_CE_HIGH()      HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_SET)

/**
 * @brief Set CE pin low
 */
#define NRF24L01_CE_LOW()       HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_RESET)

/**
 * @brief Set CSN pin high
 */
#define NRF24L01_CSN_HIGH()     HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_SET)

/**
 * @brief Set CSN pin low
 */
#define NRF24L01_CSN_LOW()      HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_RESET)

/**
 * @brief Read IRQ pin state
 * @retval 1 if IRQ is high (no interrupt), 0 if low (interrupt active)
 */
#define NRF24L01_IRQ_READ()     HAL_GPIO_ReadPin(NRF24L01_IRQ_PORT, NRF24L01_IRQ_PIN)

#ifdef __cplusplus
}
#endif

#endif /* __NRF24L01_HAL_CONFIG_H */
