/**
  ******************************************************************************
  * @file    nrf24l01.c
  * @brief   NRF24L01+ 2.4G Wireless Module Driver Implementation
  * @note    This driver supports STM32 HAL library
  *
  * @author  Mingyue Class 2026
  * @date    2026-04-25
  ******************************************************************************
  */

#include "nrf24l01.h"
#include <string.h>

/* ============================================================================
 *                          PRIVATE FUNCTIONS
 * ============================================================================ */

/**
 * @brief SPI transmit and receive a single byte
 * @param handle: NRF24L01 handle pointer
 * @param data: Data to transmit
 * @retval Received data
 */
static uint8_t NRF24L01_SPI_TransmitReceive(NRF24L01_Handle_t *handle, uint8_t data)
{
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(handle->hspi, &data, &rx_data, 1, NRF24L01_SPI_TIMEOUT);
    return rx_data;
}

/**
 * @brief SPI transmit buffer
 * @param handle: NRF24L01 handle pointer
 * @param data: Data buffer to transmit
 * @param length: Buffer length
 * @retval HAL status
 */
static HAL_StatusTypeDef NRF24L01_SPI_Transmit(NRF24L01_Handle_t *handle, uint8_t *data, uint16_t length)
{
    return HAL_SPI_Transmit(handle->hspi, data, length, NRF24L01_SPI_TIMEOUT);
}

/**
 * @brief SPI receive buffer
 * @param handle: NRF24L01 handle pointer
 * @param data: Data buffer to receive
 * @param length: Buffer length
 * @retval HAL status
 */
static HAL_StatusTypeDef NRF24L01_SPI_Receive(NRF24L01_Handle_t *handle, uint8_t *data, uint16_t length)
{
    return HAL_SPI_Receive(handle->hspi, data, length, NRF24L01_SPI_TIMEOUT);
}

/* ============================================================================
 *                          INITIALIZATION FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize NRF24L01 module
 * @param handle: NRF24L01 handle pointer
 * @param config: Initialization configuration
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_Init(NRF24L01_Handle_t *handle, NRF24L01_Init_t *config)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t config_reg = 0;

    // Store configuration
    handle->hspi = config->hspi;
    handle->ce_port = config->ce_port;
    handle->ce_pin = config->ce_pin;
    handle->csn_port = config->csn_port;
    handle->csn_pin = config->csn_pin;
    handle->irq_port = config->irq_port;
    handle->irq_pin = config->irq_pin;
    handle->payload_width = config->payload_width;

    // Initialize GPIO pins
    NRF24L01_CE_Low(handle);
    NRF24L01_CSN_High(handle);

    // Wait for power on reset
    HAL_Delay(100);

    // Flush TX and RX FIFOs
    NRF24L01_FlushTx(handle);
    NRF24L01_FlushRx(handle);

    // Clear all interrupts
    NRF24L01_WriteRegister(handle, NRF24L01_REG_STATUS,
                           NRF24L01_STATUS_RX_DR | NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);

    // Set RF channel
    NRF24L01_SetChannel(handle, config->channel);

    // Set data rate and TX power
    uint8_t rf_setup = config->data_rate | config->tx_power | 0x01; // LNA_HCURR
    NRF24L01_WriteRegister(handle, NRF24L01_REG_RF_SETUP, rf_setup);

    // Set address width
    uint8_t addr_width = NRF24L01_ADDR_WIDTH - 2; // 0x01 = 3 bytes, 0x02 = 4 bytes, 0x03 = 5 bytes
    NRF24L01_WriteRegister(handle, NRF24L01_REG_SETUP_AW, addr_width);

    // Set TX and RX address
    NRF24L01_SetAddress(handle, NRF24L01_REG_TX_ADDR, config->address, NRF24L01_ADDR_WIDTH);
    NRF24L01_SetAddress(handle, NRF24L01_REG_RX_ADDR_P0, config->address, NRF24L01_ADDR_WIDTH);

    // Set payload width
    NRF24L01_SetPayloadWidth(handle, 0, config->payload_width);

    // Configure auto acknowledgment
    if (config->auto_ack) {
        NRF24L01_WriteRegister(handle, NRF24L01_REG_EN_AA, 0x01); // Enable auto-ack on pipe 0
        NRF24L01_WriteRegister(handle, NRF24L01_REG_EN_RXADDR, 0x01); // Enable pipe 0

        // Set auto retransmit: 500us wait, 10 retransmits
        NRF24L01_WriteRegister(handle, NRF24L01_REG_SETUP_RETR, 0x1A);
    } else {
        NRF24L01_WriteRegister(handle, NRF24L01_REG_EN_AA, 0x00);
        NRF24L01_WriteRegister(handle, NRF24L01_REG_EN_RXADDR, 0x01);
    }

    // Configure dynamic payload
    if (config->dynamic_payload) {
        NRF24L01_WriteRegister(handle, NRF24L01_REG_DYNPD, 0x01); // Enable dynamic payload on pipe 0
        NRF24L01_WriteRegister(handle, NRF24L01_REG_FEATURE, 0x04); // Enable dynamic payload
    }

    // Power up and set to TX mode initially
    config_reg = NRF24L01_CONFIG_EN_CRC | NRF24L01_CONFIG_CRCO | NRF24L01_CONFIG_PWR_UP;
    NRF24L01_WriteRegister(handle, NRF24L01_REG_CONFIG, config_reg);

    handle->is_initialized = true;

    return status;
}

/**
 * @brief De-initialize NRF24L01 module
 * @param handle: NRF24L01 handle pointer
 */
void NRF24L01_DeInit(NRF24L01_Handle_t *handle)
{
    NRF24L01_PowerDown(handle);
    handle->is_initialized = false;
}

/* ============================================================================
 *                          CONFIGURATION FUNCTIONS
 * ============================================================================ */

/**
 * @brief Set RF channel
 * @param handle: NRF24L01 handle pointer
 * @param channel: RF channel (0-125)
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_SetChannel(NRF24L01_Handle_t *handle, uint8_t channel)
{
    if (channel > 125) {
        return HAL_ERROR;
    }
    return NRF24L01_WriteRegister(handle, NRF24L01_REG_RF_CH, channel);
}

/**
 * @brief Set data rate
 * @param handle: NRF24L01 handle pointer
 * @param data_rate: Data rate setting
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_SetDataRate(NRF24L01_Handle_t *handle, uint8_t data_rate)
{
    uint8_t rf_setup = NRF24L01_ReadRegister(handle, NRF24L01_REG_RF_SETUP);
    rf_setup &= ~(NRF24L01_RF_DR_250KBPS | NRF24L01_RF_DR_2MBPS);
    rf_setup |= data_rate;
    return NRF24L01_WriteRegister(handle, NRF24L01_REG_RF_SETUP, rf_setup);
}

/**
 * @brief Set TX power
 * @param handle: NRF24L01 handle pointer
 * @param tx_power: TX power setting
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_SetTxPower(NRF24L01_Handle_t *handle, uint8_t tx_power)
{
    uint8_t rf_setup = NRF24L01_ReadRegister(handle, NRF24L01_REG_RF_SETUP);
    rf_setup &= ~0x06; // Clear power bits
    rf_setup |= tx_power;
    return NRF24L01_WriteRegister(handle, NRF24L01_REG_RF_SETUP, rf_setup);
}

/**
 * @brief Set payload width for specific pipe
 * @param handle: NRF24L01 handle pointer
 * @param pipe: Pipe number (0-5)
 * @param width: Payload width (1-32 bytes)
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_SetPayloadWidth(NRF24L01_Handle_t *handle, uint8_t pipe, uint8_t width)
{
    if (pipe > 5 || width < 1 || width > 32) {
        return HAL_ERROR;
    }

    uint8_t reg = NRF24L01_REG_RX_PW_P0 + pipe;
    return NRF24L01_WriteRegister(handle, reg, width);
}

/**
 * @brief Set address for TX or RX pipe
 * @param handle: NRF24L01 handle pointer
 * @param reg: Register address
 * @param address: Address buffer
 * @param width: Address width
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_SetAddress(NRF24L01_Handle_t *handle, uint8_t reg, uint8_t *address, uint8_t width)
{
    NRF24L01_CSN_Low(handle);
    NRF24L01_SPI_TransmitReceive(handle, NRF24L01_CMD_WRITE_REG | reg);
    NRF24L01_SPI_Transmit(handle, address, width);
    NRF24L01_CSN_High(handle);

    return HAL_OK;
}

/* ============================================================================
 *                          MODE CONTROL FUNCTIONS
 * ============================================================================ */

/**
 * @brief Set module to TX mode
 * @param handle: NRF24L01 handle pointer
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_SetTxMode(NRF24L01_Handle_t *handle)
{
    NRF24L01_CE_Low(handle);

    uint8_t config = NRF24L01_ReadRegister(handle, NRF24L01_REG_CONFIG);
    config &= ~NRF24L01_CONFIG_PRIM_RX; // Set to TX mode
    config |= NRF24L01_CONFIG_PWR_UP;   // Power up
    NRF24L01_WriteRegister(handle, NRF24L01_REG_CONFIG, config);

    NRF24L01_CE_High(handle);
    HAL_Delay(1); // Wait for settling

    return HAL_OK;
}

/**
 * @brief Set module to RX mode
 * @param handle: NRF24L01 handle pointer
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_SetRxMode(NRF24L01_Handle_t *handle)
{
    NRF24L01_CE_Low(handle);

    uint8_t config = NRF24L01_ReadRegister(handle, NRF24L01_REG_CONFIG);
    config |= NRF24L01_CONFIG_PRIM_RX | NRF24L01_CONFIG_PWR_UP;
    NRF24L01_WriteRegister(handle, NRF24L01_REG_CONFIG, config);

    NRF24L01_CE_High(handle);
    HAL_Delay(1); // Wait for settling

    return HAL_OK;
}

/**
 * @brief Power down module
 * @param handle: NRF24L01 handle pointer
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_PowerDown(NRF24L01_Handle_t *handle)
{
    NRF24L01_CE_Low(handle);

    uint8_t config = NRF24L01_ReadRegister(handle, NRF24L01_REG_CONFIG);
    config &= ~NRF24L01_CONFIG_PWR_UP;
    NRF24L01_WriteRegister(handle, NRF24L01_REG_CONFIG, config);

    return HAL_OK;
}

/**
 * @brief Power up module
 * @param handle: NRF24L01 handle pointer
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_PowerUp(NRF24L01_Handle_t *handle)
{
    uint8_t config = NRF24L01_ReadRegister(handle, NRF24L01_REG_CONFIG);
    config |= NRF24L01_CONFIG_PWR_UP;
    NRF24L01_WriteRegister(handle, NRF24L01_REG_CONFIG, config);

    HAL_Delay(5); // Wait for power up

    return HAL_OK;
}

/* ============================================================================
 *                          DATA TRANSMISSION FUNCTIONS
 * ============================================================================ */

/**
 * @brief Transmit data packet
 * @param handle: NRF24L01 handle pointer
 * @param data: Data buffer to transmit
 * @param length: Data length
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_Transmit(NRF24L01_Handle_t *handle, uint8_t *data, uint8_t length)
{
    if (length > NRF24L01_MAX_PAYLOAD_WIDTH) {
        return HAL_ERROR;
    }

    // Write payload to TX FIFO
    NRF24L01_WritePayload(handle, data, length, NRF24L01_CMD_WR_TX_PAYLOAD);

    // Pulse CE to start transmission
    NRF24L01_CE_High(handle);
    HAL_Delay(1); // Keep CE high for at least 10us
    NRF24L01_CE_Low(handle);

    return HAL_OK;
}

/**
 * @brief Transmit data packet without auto-ack
 * @param handle: NRF24L01 handle pointer
 * @param data: Data buffer to transmit
 * @param length: Data length
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_TransmitNoAck(NRF24L01_Handle_t *handle, uint8_t *data, uint8_t length)
{
    if (length > NRF24L01_MAX_PAYLOAD_WIDTH) {
        return HAL_ERROR;
    }

    // Write payload to TX FIFO with no ACK
    NRF24L01_WritePayload(handle, data, length, NRF24L01_CMD_WR_TX_PAYLOAD | 0x01);

    // Pulse CE to start transmission
    NRF24L01_CE_High(handle);
    HAL_Delay(1);
    NRF24L01_CE_Low(handle);

    return HAL_OK;
}

/**
 * @brief Receive data packet
 * @param handle: NRF24L01 handle pointer
 * @param data: Data buffer to receive
 * @param length: Pointer to store received length
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_Receive(NRF24L01_Handle_t *handle, uint8_t *data, uint8_t *length)
{
    if (!NRF24L01_DataReady(handle)) {
        return HAL_ERROR;
    }

    // Read payload length (for dynamic payload)
    *length = handle->payload_width;

    // Read payload from RX FIFO
    NRF24L01_ReadPayload(handle, data, *length);

    // Clear RX_DR interrupt
    NRF24L01_WriteRegister(handle, NRF24L01_REG_STATUS, NRF24L01_STATUS_RX_DR);

    return HAL_OK;
}

/* ============================================================================
 *                          STATUS FUNCTIONS
 * ============================================================================ */

/**
 * @brief Get module status
 * @param handle: NRF24L01 handle pointer
 * @retval Status structure
 */
NRF24L01_Status_t NRF24L01_GetStatus(NRF24L01_Handle_t *handle)
{
    NRF24L01_Status_t status;
    uint8_t status_reg = NRF24L01_ReadRegister(handle, NRF24L01_REG_STATUS);

    status.rx_dr = (status_reg & NRF24L01_STATUS_RX_DR) ? 1 : 0;
    status.tx_ds = (status_reg & NRF24L01_STATUS_TX_DS) ? 1 : 0;
    status.max_rt = (status_reg & NRF24L01_STATUS_MAX_RT) ? 1 : 0;
    status.rx_p_no = (status_reg & NRF24L01_STATUS_RX_P_NO) >> 1;
    status.tx_full = (status_reg & NRF24L01_STATUS_TX_FULL) ? 1 : 0;

    return status;
}

/**
 * @brief Get FIFO status
 * @param handle: NRF24L01 handle pointer
 * @retval FIFO status structure
 */
NRF24L01_FifoStatus_t NRF24L01_GetFifoStatus(NRF24L01_Handle_t *handle)
{
    NRF24L01_FifoStatus_t fifo_status;
    uint8_t fifo_reg = NRF24L01_ReadRegister(handle, NRF24L01_REG_FIFO_STATUS);

    fifo_status.tx_full = (fifo_reg & NRF24L01_FIFO_TX_FULL) ? 1 : 0;
    fifo_status.tx_empty = (fifo_reg & NRF24L01_FIFO_TX_EMPTY) ? 1 : 0;
    fifo_status.rx_full = (fifo_reg & NRF24L01_FIFO_RX_FULL) ? 1 : 0;
    fifo_status.rx_empty = (fifo_reg & NRF24L01_FIFO_RX_EMPTY) ? 1 : 0;

    return fifo_status;
}

/**
 * @brief Check if data is ready in RX FIFO
 * @param handle: NRF24L01 handle pointer
 * @retval 1 if data ready, 0 otherwise
 */
uint8_t NRF24L01_DataReady(NRF24L01_Handle_t *handle)
{
    NRF24L01_FifoStatus_t fifo_status = NRF24L01_GetFifoStatus(handle);
    return !fifo_status.rx_empty;
}

/**
 * @brief Check if module is currently sending
 * @param handle: NRF24L01 handle pointer
 * @retval 1 if sending, 0 otherwise
 */
uint8_t NRF24L01_IsSending(NRF24L01_Handle_t *handle)
{
    NRF24L01_Status_t status = NRF24L01_GetStatus(handle);

    if (status.tx_ds || status.max_rt) {
        return 0; // Transmission complete
    }

    return 1; // Still sending
}

/* ============================================================================
 *                          FIFO CONTROL FUNCTIONS
 * ============================================================================ */

/**
 * @brief Flush TX FIFO
 * @param handle: NRF24L01 handle pointer
 */
void NRF24L01_FlushTx(NRF24L01_Handle_t *handle)
{
    NRF24L01_CSN_Low(handle);
    NRF24L01_SPI_TransmitReceive(handle, NRF24L01_CMD_FLUSH_TX);
    NRF24L01_CSN_High(handle);
}

/**
 * @brief Flush RX FIFO
 * @param handle: NRF24L01 handle pointer
 */
void NRF24L01_FlushRx(NRF24L01_Handle_t *handle)
{
    NRF24L01_CSN_Low(handle);
    NRF24L01_SPI_TransmitReceive(handle, NRF24L01_CMD_FLUSH_RX);
    NRF24L01_CSN_High(handle);
}

/* ============================================================================
 *                          LOW-LEVEL FUNCTIONS
 * ============================================================================ */

/**
 * @brief Read register value
 * @param handle: NRF24L01 handle pointer
 * @param reg: Register address
 * @retval Register value
 */
uint8_t NRF24L01_ReadRegister(NRF24L01_Handle_t *handle, uint8_t reg)
{
    uint8_t value;

    NRF24L01_CSN_Low(handle);
    NRF24L01_SPI_TransmitReceive(handle, NRF24L01_CMD_READ_REG | reg);
    value = NRF24L01_SPI_TransmitReceive(handle, NRF24L01_CMD_NOP);
    NRF24L01_CSN_High(handle);

    return value;
}

/**
 * @brief Write register value
 * @param handle: NRF24L01 handle pointer
 * @param reg: Register address
 * @param value: Value to write
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_WriteRegister(NRF24L01_Handle_t *handle, uint8_t reg, uint8_t value)
{
    NRF24L01_CSN_Low(handle);
    NRF24L01_SPI_TransmitReceive(handle, NRF24L01_CMD_WRITE_REG | reg);
    NRF24L01_SPI_TransmitReceive(handle, value);
    NRF24L01_CSN_High(handle);

    return HAL_OK;
}

/**
 * @brief Read payload from RX FIFO
 * @param handle: NRF24L01 handle pointer
 * @param data: Data buffer
 * @param length: Data length
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_ReadPayload(NRF24L01_Handle_t *handle, uint8_t *data, uint8_t length)
{
    NRF24L01_CSN_Low(handle);
    NRF24L01_SPI_TransmitReceive(handle, NRF24L01_CMD_RD_RX_PAYLOAD);
    NRF24L01_SPI_Receive(handle, data, length);
    NRF24L01_CSN_High(handle);

    return HAL_OK;
}

/**
 * @brief Write payload to TX FIFO
 * @param handle: NRF24L01 handle pointer
 * @param data: Data buffer
 * @param length: Data length
 * @param cmd: Command byte
 * @retval HAL status
 */
HAL_StatusTypeDef NRF24L01_WritePayload(NRF24L01_Handle_t *handle, uint8_t *data, uint8_t length, uint8_t cmd)
{
    NRF24L01_CSN_Low(handle);
    NRF24L01_SPI_TransmitReceive(handle, cmd);
    NRF24L01_SPI_Transmit(handle, data, length);
    NRF24L01_CSN_High(handle);

    return HAL_OK;
}

/* ============================================================================
 *                          UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * @brief Set CE pin high
 * @param handle: NRF24L01 handle pointer
 */
void NRF24L01_CE_High(NRF24L01_Handle_t *handle)
{
    HAL_GPIO_WritePin(handle->ce_port, handle->ce_pin, GPIO_PIN_SET);
}

/**
 * @brief Set CE pin low
 * @param handle: NRF24L01 handle pointer
 */
void NRF24L01_CE_Low(NRF24L01_Handle_t *handle)
{
    HAL_GPIO_WritePin(handle->ce_port, handle->ce_pin, GPIO_PIN_RESET);
}

/**
 * @brief Set CSN pin high
 * @param handle: NRF24L01 handle pointer
 */
void NRF24L01_CSN_High(NRF24L01_Handle_t *handle)
{
    HAL_GPIO_WritePin(handle->csn_port, handle->csn_pin, GPIO_PIN_SET);
}

/**
 * @brief Set CSN pin low
 * @param handle: NRF24L01 handle pointer
 */
void NRF24L01_CSN_Low(NRF24L01_Handle_t *handle)
{
    HAL_GPIO_WritePin(handle->csn_port, handle->csn_pin, GPIO_PIN_RESET);
}
