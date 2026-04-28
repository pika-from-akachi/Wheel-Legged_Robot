/**
  ******************************************************************************
  * @file    nrf24l01_rx.c
  * @brief   NRF24L01+ Receiver Driver Implementation
  * @note    Simplified driver for receiver only
  *
  * @author  Mingyue Class 2026
  * @date    2026-04-25
  ******************************************************************************
  */

#include "nrf24l01_rx.h"
#include <string.h>

/* Global receiver handle */
NRF24L01_RX_Handle_t nrf_rx_handle = {0};

/* ============================================================================
 *                          GPIO INITIALIZATION
 * ============================================================================ */

/**
 * @brief Initialize GPIO pins for NRF24L01
 */
void NRF24L01_RX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIOC clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure CE Pin (PC8) - Output */
    GPIO_InitStruct.Pin = NRF24L01_CE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStruct);

    /* Configure CSN Pin (PC9) - Output */
    GPIO_InitStruct.Pin = NRF24L01_CSN_PIN;
    HAL_GPIO_Init(NRF24L01_CSN_PORT, &GPIO_InitStruct);

    /* Configure IRQ Pin (PC7) - Input (optional) */
    GPIO_InitStruct.Pin = NRF24L01_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NRF24L01_IRQ_PORT, &GPIO_InitStruct);

    /* Set initial states */
    NRF24L01_CE_LOW();
    NRF24L01_CSN_HIGH();
}

/* ============================================================================
 *                          SPI FUNCTIONS
 * ============================================================================ */

/**
 * @brief SPI transmit and receive a single byte
 */
static uint8_t NRF24L01_RX_SPI_TransmitReceive(uint8_t data)
{
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(&hspi3, &data, &rx_data, 1, NRF24L01_SPI_TIMEOUT);
    return rx_data;
}

/**
 * @brief Read register value
 */
uint8_t NRF24L01_RX_ReadRegister(uint8_t reg)
{
    uint8_t value;

    NRF24L01_CSN_LOW();
    NRF24L01_RX_SPI_TransmitReceive(NRF24L01_CMD_READ_REG | reg);
    value = NRF24L01_RX_SPI_TransmitReceive(NRF24L01_CMD_NOP);
    NRF24L01_CSN_HIGH();

    return value;
}

/**
 * @brief Write register value
 */
void NRF24L01_RX_WriteRegister(uint8_t reg, uint8_t value)
{
    NRF24L01_CSN_LOW();
    NRF24L01_RX_SPI_TransmitReceive(NRF24L01_CMD_WRITE_REG | reg);
    NRF24L01_RX_SPI_TransmitReceive(value);
    NRF24L01_CSN_HIGH();
}

/**
 * @brief Flush RX FIFO
 */
void NRF24L01_RX_FlushRx(void)
{
    NRF24L01_CSN_LOW();
    NRF24L01_RX_SPI_TransmitReceive(NRF24L01_CMD_FLUSH_RX);
    NRF24L01_CSN_HIGH();
}

/* ============================================================================
 *                          INITIALIZATION
 * ============================================================================ */

/**
 * @brief Initialize NRF24L01 as receiver with default pairing address "HXFB0"
 */
void NRF24L01_RX_Init(void)
{
    // Initialize GPIO
    NRF24L01_RX_GPIO_Init();

    // Store SPI handle
    nrf_rx_handle.hspi = &hspi3;
    nrf_rx_handle.payload_width = 16; // 16 bytes payload as per datasheet

    // Wait for power on
    HAL_Delay(100);

    // Flush RX FIFO
    NRF24L01_RX_FlushRx();

    // Clear interrupts
    NRF24L01_RX_WriteRegister(NRF24L01_REG_STATUS,
                              NRF24L01_STATUS_RX_DR | NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);

    // Set RF channel to CH25 (2.425GHz) as per datasheet
    NRF24L01_RX_WriteRegister(NRF24L01_REG_RF_CH, 25);

    // Set data rate: 250kbps, TX power: 0dBm (as per datasheet: 250kbps air rate)
    NRF24L01_RX_WriteRegister(NRF24L01_REG_RF_SETUP, 0x20 | NRF24L01_RF_PWR_0DBM | 0x01); // 250kbps

    // Set address width: 5 bytes
    NRF24L01_RX_WriteRegister(NRF24L01_REG_SETUP_AW, 0x03);

    // Set default RX address to "HXFB0" (0x48, 0x58, 0x46, 0x42, 0x30)
    uint8_t default_address[5] = {0x48, 0x58, 0x46, 0x42, 0x30}; // "HXFB0"
    NRF24L01_RX_SetAddress(default_address);

    // Set payload width to 16 bytes
    NRF24L01_RX_WriteRegister(NRF24L01_REG_RX_PW_P0, nrf_rx_handle.payload_width);

    // Enable auto-ack on pipe 0
    NRF24L01_RX_WriteRegister(NRF24L01_REG_EN_AA, 0x01);

    // Enable pipe 0
    NRF24L01_RX_WriteRegister(NRF24L01_REG_EN_RXADDR, 0x01);

    // Set to RX mode, power up, enable CRC
    NRF24L01_RX_WriteRegister(NRF24L01_REG_CONFIG, 0x0F);

    // Set CE high to start listening
    NRF24L01_CE_HIGH();

    // Wait for settling
    HAL_Delay(1);

    nrf_rx_handle.is_initialized = 1;
    nrf_rx_handle.is_paired = 0; // Not paired yet
}

/* ============================================================================
 *                          DATA RECEPTION
 * ============================================================================ */

/**
 * @brief Check if data is available in RX FIFO
 * @retval 1 if data ready, 0 otherwise
 */
uint8_t NRF24L01_RX_CheckData(void)
{
    uint8_t status = NRF24L01_RX_ReadRegister(NRF24L01_REG_STATUS);
    return (status & NRF24L01_STATUS_RX_DR) ? 1 : 0;
}

/**
 * @brief Set RX address (also sets TX address for auto-ack)
 * @param address: 5-byte address array
 */
void NRF24L01_RX_SetAddress(uint8_t *address)
{
    // Set RX address for pipe 0
    NRF24L01_CSN_LOW();
    NRF24L01_RX_SPI_TransmitReceive(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_RX_ADDR_P0);
    for (uint8_t i = 0; i < 5; i++) {
        NRF24L01_RX_SPI_TransmitReceive(address[i]);
        nrf_rx_handle.paired_address[i] = address[i];
    }
    NRF24L01_CSN_HIGH();

    // Set TX address (must match RX_ADDR_P0 for auto-ack)
    NRF24L01_CSN_LOW();
    NRF24L01_RX_SPI_TransmitReceive(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_TX_ADDR);
    for (uint8_t i = 0; i < 5; i++) {
        NRF24L01_RX_SPI_TransmitReceive(address[i]);
    }
    NRF24L01_CSN_HIGH();
}

/**
 * @brief Wait for pairing with remote controller
 * @retval 1 if paired successfully, 0 if timeout
 * @note    Pairing process according to datasheet:
 *          1. Device starts with default address "HXFB0"
 *          2. Remote sends data to "HXFB0"
 *          3. Device receives and sends ACK with new address
 *          4. Both switch to new address
 */
uint8_t NRF24L01_RX_WaitForPairing(void)
{
    uint32_t start_time = HAL_GetTick();
    uint8_t tx_buffer[16];

    // Wait for first data packet from remote (timeout 10 seconds)
    while ((HAL_GetTick() - start_time) < 10000) {
        if (NRF24L01_RX_CheckData()) {
            // Read the packet
            NRF24L01_CSN_LOW();
            NRF24L01_RX_SPI_TransmitReceive(NRF24L01_CMD_RD_RX_PAYLOAD);
            for (uint8_t i = 0; i < 16; i++) {
                nrf_rx_handle.rx_buffer[i] = NRF24L01_RX_SPI_TransmitReceive(NRF24L01_CMD_NOP);
            }
            NRF24L01_CSN_HIGH();

            // Clear RX_DR interrupt
            NRF24L01_RX_WriteRegister(NRF24L01_REG_STATUS, NRF24L01_STATUS_RX_DR);

            // Validate packet format (check fixed values)
            if (nrf_rx_handle.rx_buffer[0] == 0x01 &&
                nrf_rx_handle.rx_buffer[1] == 0x03 &&
                nrf_rx_handle.rx_buffer[3] == 0x11) {

                // Store rolling code
                nrf_rx_handle.rolling_code = nrf_rx_handle.rx_buffer[2];

                // Generate a new pairing address
                // Use timestamp-based pseudo-random
                uint32_t random_val = HAL_GetTick() ^ 0x5A5A5A5A;
                uint8_t new_address[5];
                new_address[0] = 0x48 + (random_val % 10);
                new_address[1] = 0x58 + ((random_val >> 8) % 10);
                new_address[2] = 0x46 + ((random_val >> 16) % 10);
                new_address[3] = 0x42 + ((random_val >> 24) % 10);
                new_address[4] = 0x30 + ((random_val >> 4) % 10);

                // Prepare response packet (16 bytes) according to datasheet
                tx_buffer[0] = 0x01;           // Fixed value 1
                tx_buffer[1] = 0x83;           // Fixed value 0x83
                tx_buffer[2] = nrf_rx_handle.rolling_code; // Rolling code (same as received)
                tx_buffer[3] = 0x11;           // Fixed value 11
                tx_buffer[4] = new_address[0]; // Pairing address 0
                tx_buffer[5] = new_address[1]; // Pairing address 1
                tx_buffer[6] = new_address[2]; // Pairing address 2
                tx_buffer[7] = new_address[3]; // Pairing address 3
                tx_buffer[8] = new_address[4]; // Pairing address 4
                tx_buffer[9] = 0x00;           // Fixed 0
                tx_buffer[10] = 0x00;          // Fixed 0
                tx_buffer[11] = 0x00;          // Fixed 0
                tx_buffer[12] = 0x00;          // Fixed 0
                tx_buffer[13] = 0x00;          // Fixed 0
                tx_buffer[14] = 0x00;          // Fixed 0
                // Calculate checksum (sum of bytes 0-14)
                tx_buffer[15] = 0;
                for (uint8_t i = 0; i < 15; i++) {
                    tx_buffer[15] += tx_buffer[i];
                }

                // IMPORTANT: Switch to TX mode while still using "HXFB0" address
                // The remote is still listening on "HXFB0"
                NRF24L01_CE_LOW();
                NRF24L01_RX_WriteRegister(NRF24L01_REG_CONFIG, 0x0E); // TX mode, PWR_UP, CRC enabled
                HAL_Delay(1);

                // Write TX payload
                NRF24L01_CSN_LOW();
                NRF24L01_RX_SPI_TransmitReceive(0xA0); // WR_TX_PAYLOAD
                for (uint8_t i = 0; i < 16; i++) {
                    NRF24L01_RX_SPI_TransmitReceive(tx_buffer[i]);
                }
                NRF24L01_CSN_HIGH();

                // Start transmission (CE high for at least 10us)
                NRF24L01_CE_HIGH();
                HAL_Delay(1); // Wait for transmission
                NRF24L01_CE_LOW();

                // Wait for TX complete or timeout
                uint32_t tx_start = HAL_GetTick();
                uint8_t tx_complete = 0;
                while ((HAL_GetTick() - tx_start) < 100) { // 100ms timeout
                    uint8_t status = NRF24L01_RX_ReadRegister(NRF24L01_REG_STATUS);
                    if (status & NRF24L01_STATUS_TX_DS) {
                        // TX success
                        tx_complete = 1;
                        break;
                    }
                    if (status & NRF24L01_STATUS_MAX_RT) {
                        // Max retransmits reached
                        break;
                    }
                    HAL_Delay(1);
                }

                // Clear TX interrupts
                NRF24L01_RX_WriteRegister(NRF24L01_REG_STATUS,
                                         NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);

                // Now switch to new address and RX mode
                NRF24L01_RX_SetAddress(new_address);
                NRF24L01_RX_WriteRegister(NRF24L01_REG_CONFIG, 0x0F); // RX mode
                NRF24L01_CE_HIGH();
                HAL_Delay(1);

                nrf_rx_handle.is_paired = 1;
                return 1; // Paired successfully
            }
        }
        HAL_Delay(10);
    }

    return 0; // Timeout
}

/**
 * @brief Check if receiver is paired with remote
 * @retval 1 if paired, 0 otherwise
 */
uint8_t NRF24L01_RX_IsPaired(void)
{
    return nrf_rx_handle.is_paired;
}

/**
 * @brief Read and parse received data according to datasheet protocol
 * @retval 1 if data received successfully, 0 otherwise
 */
uint8_t NRF24L01_RX_ReadData(void)
{
    if (!NRF24L01_RX_CheckData()) {
        return 0;
    }

    // Read payload from RX FIFO
    NRF24L01_CSN_LOW();
    NRF24L01_RX_SPI_TransmitReceive(NRF24L01_CMD_RD_RX_PAYLOAD);

    for (uint8_t i = 0; i < nrf_rx_handle.payload_width; i++) {
        nrf_rx_handle.rx_buffer[i] = NRF24L01_RX_SPI_TransmitReceive(NRF24L01_CMD_NOP);
    }
    NRF24L01_CSN_HIGH();

    // Clear RX_DR interrupt
    NRF24L01_RX_WriteRegister(NRF24L01_REG_STATUS, NRF24L01_STATUS_RX_DR);

    // Validate packet format according to datasheet
    // Byte 0: Fixed value 1
    // Byte 1: Fixed value 3
    // Byte 3: Fixed value 11
    if (nrf_rx_handle.rx_buffer[0] != 0x01 ||
        nrf_rx_handle.rx_buffer[1] != 0x03 ||
        nrf_rx_handle.rx_buffer[3] != 0x11) {
        return 0; // Invalid packet
    }

    // Verify checksum (sum of bytes 0-14 should equal byte 15)
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 15; i++) {
        checksum += nrf_rx_handle.rx_buffer[i];
    }
    if (checksum != nrf_rx_handle.rx_buffer[15]) {
        return 0; // Checksum error
    }

    // Parse data according to datasheet protocol
    nrf_rx_handle.rolling_code = nrf_rx_handle.rx_buffer[2];
    nrf_rx_handle.rc_data.right_joystick_x = nrf_rx_handle.rx_buffer[4];
    nrf_rx_handle.rc_data.right_joystick_y = nrf_rx_handle.rx_buffer[5];
    nrf_rx_handle.rc_data.left_joystick_x = nrf_rx_handle.rx_buffer[6];
    nrf_rx_handle.rc_data.left_joystick_y = nrf_rx_handle.rx_buffer[7];
    nrf_rx_handle.rc_data.button_state = nrf_rx_handle.rx_buffer[8];
    nrf_rx_handle.rc_data.rolling_code = nrf_rx_handle.rx_buffer[2];
    nrf_rx_handle.rc_data.data_valid = 1;
    nrf_rx_handle.rc_data.timestamp = HAL_GetTick();
    nrf_rx_handle.last_receive_time = HAL_GetTick();

    return 1;
}

/**
 * @brief Get parsed remote control data
 * @retval Pointer to remote control data structure
 */
RemoteControlData_t* NRF24L01_RX_GetData(void)
{
    return &nrf_rx_handle.rc_data;
}

/* ============================================================================
 *                          STATUS FUNCTIONS
 * ============================================================================ */

/**
 * @brief Check if receiver is online (received data recently)
 * @retval 1 if online (data received within last 500ms), 0 otherwise
 */
uint8_t NRF24L01_RX_IsOnline(void)
{
    if (nrf_rx_handle.last_receive_time == 0) {
        return 0;
    }

    uint32_t elapsed = HAL_GetTick() - nrf_rx_handle.last_receive_time;
    return (elapsed < 500) ? 1 : 0; // 500ms timeout
}

/**
 * @brief Get last receive timestamp
 * @retval Last receive time in milliseconds
 */
uint32_t NRF24L01_RX_GetLastReceiveTime(void)
{
    return nrf_rx_handle.last_receive_time;
}
