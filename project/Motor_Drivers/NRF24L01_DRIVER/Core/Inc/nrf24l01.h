/**
  ******************************************************************************
  * @file    nrf24l01.h
  * @brief   NRF24L01+ 2.4G Wireless Module Driver Header
  * @note    This driver supports STM32 HAL library
  *
  * @author  Mingyue Class 2026
  * @date    2026-04-25
  ******************************************************************************
  */

#ifndef __NRF24L01_H
#define __NRF24L01_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *                          CONFIGURATION
 * ============================================================================ */

/* SPI Configuration */
#define NRF24L01_SPI_TIMEOUT        100     // SPI timeout in ms

/* Payload Configuration */
#define NRF24L01_MAX_PAYLOAD_WIDTH  32      // Maximum payload width
#define NRF24L01_DEFAULT_CHANNEL    70      // Default RF channel (2.400GHz + channel * 1MHz)

/* Address Configuration */
#define NRF24L01_ADDR_WIDTH         5       // Address width: 3-5 bytes

/* ============================================================================
 *                          REGISTER DEFINITIONS
 * ============================================================================ */

/* NRF24L01 Commands */
#define NRF24L01_CMD_READ_REG       0x00    // Read register command
#define NRF24L01_CMD_WRITE_REG      0x20    // Write register command
#define NRF24L01_CMD_RD_RX_PAYLOAD  0x61    // Read RX payload
#define NRF24L01_CMD_WR_TX_PAYLOAD  0xA0    // Write TX payload
#define NRF24L01_CMD_FLUSH_TX       0xE1    // Flush TX FIFO
#define NRF24L01_CMD_FLUSH_RX       0xE2    // Flush RX FIFO
#define NRF24L01_CMD_REUSE_TX_PL    0xE3    // Reuse TX payload
#define NRF24L01_CMD_NOP            0xFF    // No operation (read STATUS)

/* NRF24L01 Registers */
#define NRF24L01_REG_CONFIG         0x00    // Configuration register
#define NRF24L01_REG_EN_AA          0x01    // Enable auto acknowledgment
#define NRF24L01_REG_EN_RXADDR      0x02    // Enabled RX addresses
#define NRF24L01_REG_SETUP_AW       0x03    // Setup address width
#define NRF24L01_REG_SETUP_RETR     0x04    // Setup auto retransmit
#define NRF24L01_REG_RF_CH          0x05    // RF channel
#define NRF24L01_REG_RF_SETUP       0x06    // RF setup register
#define NRF24L01_REG_STATUS         0x07    // Status register
#define NRF24L01_REG_OBSERVE_TX     0x08    // Transmit observe register
#define NRF24L01_REG_RPD            0x09    // Received power detector
#define NRF24L01_REG_RX_ADDR_P0     0x0A    // RX address pipe 0
#define NRF24L01_REG_RX_ADDR_P1     0x0B    // RX address pipe 1
#define NRF24L01_REG_RX_ADDR_P2     0x0C    // RX address pipe 2
#define NRF24L01_REG_RX_ADDR_P3     0x0D    // RX address pipe 3
#define NRF24L01_REG_RX_ADDR_P4     0x0E    // RX address pipe 4
#define NRF24L01_REG_RX_ADDR_P5     0x0F    // RX address pipe 5
#define NRF24L01_REG_TX_ADDR        0x10    // TX address
#define NRF24L01_REG_RX_PW_P0       0x11    // RX payload width pipe 0
#define NRF24L01_REG_RX_PW_P1       0x12    // RX payload width pipe 1
#define NRF24L01_REG_RX_PW_P2       0x13    // RX payload width pipe 2
#define NRF24L01_REG_RX_PW_P3       0x14    // RX payload width pipe 3
#define NRF24L01_REG_RX_PW_P4       0x15    // RX payload width pipe 4
#define NRF24L01_REG_RX_PW_P5       0x16    // RX payload width pipe 5
#define NRF24L01_REG_FIFO_STATUS    0x17    // FIFO status register
#define NRF24L01_REG_DYNPD          0x1C    // Dynamic payload enable
#define NRF24L01_REG_FEATURE        0x1D    // Feature register

/* CONFIG Register Bits */
#define NRF24L01_CONFIG_MASK_RX_DR  (1 << 6)    // Mask RX_DR interrupt
#define NRF24L01_CONFIG_MASK_TX_DS  (1 << 5)    // Mask TX_DS interrupt
#define NRF24L01_CONFIG_MASK_MAX_RT (1 << 4)    // Mask MAX_RT interrupt
#define NRF24L01_CONFIG_EN_CRC      (1 << 3)    // Enable CRC
#define NRF24L01_CONFIG_CRCO        (1 << 2)    // CRC encoding (0: 1 byte, 1: 2 bytes)
#define NRF24L01_CONFIG_PWR_UP      (1 << 1)    // Power up
#define NRF24L01_CONFIG_PRIM_RX     (1 << 0)    // RX/TX control (0: TX, 1: RX)

/* STATUS Register Bits */
#define NRF24L01_STATUS_RX_DR       (1 << 6)    // Data ready RX FIFO interrupt
#define NRF24L01_STATUS_TX_DS       (1 << 5)    // Data sent TX FIFO interrupt
#define NRF24L01_STATUS_MAX_RT      (1 << 4)    // Maximum TX retransmits interrupt
#define NRF24L01_STATUS_RX_P_NO     (0x07 << 1) // RX pipe number (bits 3:1)
#define NRF24L01_STATUS_TX_FULL     (1 << 0)    // TX FIFO full

/* FIFO_STATUS Register Bits */
#define NRF24L01_FIFO_TX_REUSE      (1 << 6)    // TX reuse
#define NRF24L01_FIFO_TX_FULL       (1 << 5)    // TX FIFO full
#define NRF24L01_FIFO_TX_EMPTY      (1 << 4)    // TX FIFO empty
#define NRF24L01_FIFO_RX_FULL       (1 << 1)    // RX FIFO full
#define NRF24L01_FIFO_RX_EMPTY      (1 << 0)    // RX FIFO empty

/* RF_SETUP Register Bits */
#define NRF24L01_RF_SETUP_CONT_WAVE (1 << 7)    // Continuous carrier transmit
#define NRF24L01_RF_SETUP_RF_DR_LOW (1 << 5)    // Set RF Data Rate to 250kbps
#define NRF24L01_RF_SETUP_PLL_LOCK  (1 << 4)    // Force PLL lock signal
#define NRF24L01_RF_SETUP_RF_DR_HIGH (1 << 3)   // Select high speed data rate

/* RF Data Rate Settings */
#define NRF24L01_RF_DR_250KBPS      0x20        // 250kbps
#define NRF24L01_RF_DR_1MBPS        0x00        // 1Mbps
#define NRF24L01_RF_DR_2MBPS        0x08        // 2Mbps

/* RF Output Power Settings */
#define NRF24L01_RF_PWR_NEG18DBM    0x00        // -18dBm
#define NRF24L01_RF_PWR_NEG12DBM    0x02        // -12dBm
#define NRF24L01_RF_PWR_NEG6DBM     0x04        // -6dBm
#define NRF24L01_RF_PWR_0DBM        0x06        // 0dBm

/* ============================================================================
 *                          TYPE DEFINITIONS
 * ============================================================================ */

/**
 * @brief NRF24L01 initialization structure
 */
typedef struct {
    SPI_HandleTypeDef *hspi;        // SPI handle
    GPIO_TypeDef *ce_port;          // CE pin port
    uint16_t ce_pin;                // CE pin number
    GPIO_TypeDef *csn_port;         // CSN pin port
    uint16_t csn_pin;               // CSN pin number
    GPIO_TypeDef *irq_port;         // IRQ pin port (optional)
    uint16_t irq_pin;               // IRQ pin number (optional)
    uint8_t channel;                // RF channel (0-125)
    uint8_t payload_width;          // Payload width (1-32 bytes)
    uint8_t data_rate;              // Data rate (250kbps/1Mbps/2Mbps)
    uint8_t tx_power;               // TX power (0/-6/-12/-18 dBm)
    uint8_t address[5];             // TX/RX address
    bool auto_ack;                  // Enable auto acknowledgment
    bool dynamic_payload;           // Enable dynamic payload
} NRF24L01_Init_t;

/**
 * @brief NRF24L01 handle structure
 */
typedef struct {
    SPI_HandleTypeDef *hspi;        // SPI handle
    GPIO_TypeDef *ce_port;          // CE pin port
    uint16_t ce_pin;                // CE pin number
    GPIO_TypeDef *csn_port;         // CSN pin port
    uint16_t csn_pin;               // CSN pin number
    GPIO_TypeDef *irq_port;         // IRQ pin port
    uint16_t irq_pin;               // IRQ pin number
    uint8_t payload_width;          // Current payload width
    bool is_initialized;            // Initialization flag
} NRF24L01_Handle_t;

/**
 * @brief NRF24L01 status structure
 */
typedef struct {
    uint8_t rx_dr;                  // RX data ready flag
    uint8_t tx_ds;                  // TX data sent flag
    uint8_t max_rt;                 // Max retransmit flag
    uint8_t rx_p_no;                // RX pipe number
    uint8_t tx_full;                // TX FIFO full flag
} NRF24L01_Status_t;

/**
 * @brief NRF24L01 FIFO status structure
 */
typedef struct {
    uint8_t tx_full;                // TX FIFO full
    uint8_t tx_empty;               // TX FIFO empty
    uint8_t rx_full;                // RX FIFO full
    uint8_t rx_empty;               // RX FIFO empty
} NRF24L01_FifoStatus_t;

/* ============================================================================
 *                          FUNCTION PROTOTYPES
 * ============================================================================ */

/* Initialization Functions */
HAL_StatusTypeDef NRF24L01_Init(NRF24L01_Handle_t *handle, NRF24L01_Init_t *config);
void NRF24L01_DeInit(NRF24L01_Handle_t *handle);

/* Configuration Functions */
HAL_StatusTypeDef NRF24L01_SetChannel(NRF24L01_Handle_t *handle, uint8_t channel);
HAL_StatusTypeDef NRF24L01_SetDataRate(NRF24L01_Handle_t *handle, uint8_t data_rate);
HAL_StatusTypeDef NRF24L01_SetTxPower(NRF24L01_Handle_t *handle, uint8_t tx_power);
HAL_StatusTypeDef NRF24L01_SetPayloadWidth(NRF24L01_Handle_t *handle, uint8_t pipe, uint8_t width);
HAL_StatusTypeDef NRF24L01_SetAddress(NRF24L01_Handle_t *handle, uint8_t reg, uint8_t *address, uint8_t width);

/* Mode Control Functions */
HAL_StatusTypeDef NRF24L01_SetTxMode(NRF24L01_Handle_t *handle);
HAL_StatusTypeDef NRF24L01_SetRxMode(NRF24L01_Handle_t *handle);
HAL_StatusTypeDef NRF24L01_PowerDown(NRF24L01_Handle_t *handle);
HAL_StatusTypeDef NRF24L01_PowerUp(NRF24L01_Handle_t *handle);

/* Data Transmission Functions */
HAL_StatusTypeDef NRF24L01_Transmit(NRF24L01_Handle_t *handle, uint8_t *data, uint8_t length);
HAL_StatusTypeDef NRF24L01_TransmitNoAck(NRF24L01_Handle_t *handle, uint8_t *data, uint8_t length);
HAL_StatusTypeDef NRF24L01_Receive(NRF24L01_Handle_t *handle, uint8_t *data, uint8_t *length);

/* Status Functions */
NRF24L01_Status_t NRF24L01_GetStatus(NRF24L01_Handle_t *handle);
NRF24L01_FifoStatus_t NRF24L01_GetFifoStatus(NRF24L01_Handle_t *handle);
uint8_t NRF24L01_DataReady(NRF24L01_Handle_t *handle);
uint8_t NRF24L01_IsSending(NRF24L01_Handle_t *handle);

/* FIFO Control Functions */
void NRF24L01_FlushTx(NRF24L01_Handle_t *handle);
void NRF24L01_FlushRx(NRF24L01_Handle_t *handle);

/* Low-Level Functions */
uint8_t NRF24L01_ReadRegister(NRF24L01_Handle_t *handle, uint8_t reg);
HAL_StatusTypeDef NRF24L01_WriteRegister(NRF24L01_Handle_t *handle, uint8_t reg, uint8_t value);
HAL_StatusTypeDef NRF24L01_ReadPayload(NRF24L01_Handle_t *handle, uint8_t *data, uint8_t length);
HAL_StatusTypeDef NRF24L01_WritePayload(NRF24L01_Handle_t *handle, uint8_t *data, uint8_t length, uint8_t cmd);

/* Utility Functions */
void NRF24L01_CE_High(NRF24L01_Handle_t *handle);
void NRF24L01_CE_Low(NRF24L01_Handle_t *handle);
void NRF24L01_CSN_High(NRF24L01_Handle_t *handle);
void NRF24L01_CSN_Low(NRF24L01_Handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* __NRF24L01_H */
