/**
  ******************************************************************************
  * @file    nrf24l01_rx.c
  * @brief   NRF24L01+ Receiver Driver - Exact match to hu_m40.c example
  ******************************************************************************
  */

#include "nrf24l01_rx.h"
#include <string.h>

#define PAYLOAD_WIDTH 16

/* Global receiver handle */
NRF24L01_RX_Handle_t nrf_rx_handle = {0};

/* Default address "HXFB0" */
static const uint8_t HU_M40_ADDR_DEF[5] = "HXFB0";

/* New address after pairing "HXFB1" */
static uint8_t g_hu_m40_rx_addr[5] = "HXFB1";

/* Current TX address pointer */
static uint8_t* g_hu_m40_tx_addr;

/* RX data buffer */
static uint8_t g_hu_m40_rx_data[16];

/* Debug variables */
volatile uint8_t g_ack_payload_sent = 0;
volatile uint8_t g_fallback_tx_used = 0;
volatile uint8_t g_debug_en_aa = 0;
volatile uint8_t g_debug_en_rxaddr = 0;
volatile uint8_t g_debug_rx_pw_p0 = 0;
volatile uint8_t g_debug_feature = 0;
volatile uint8_t g_debug_dynpd = 0;
volatile uint8_t g_debug_config = 0;
volatile uint8_t g_debug_status = 0;
volatile uint8_t g_debug_fifo_status = 0;
volatile uint8_t g_debug_rx_dr_count = 0;
volatile uint8_t g_debug_ce_state = 0;
volatile uint8_t g_debug_rx_addr[5] = {0};
volatile uint8_t g_debug_tx_addr[5] = {0};
volatile uint8_t g_debug_first_packet[4] = {0};
volatile uint8_t g_debug_packets_on_new_addr = 0;
volatile uint8_t g_debug_last_rolling_code = 0;
volatile uint8_t g_debug_observe_tx = 0;
volatile uint8_t g_debug_arc_cnt = 0;
volatile uint8_t g_debug_rf_setup = 0;
volatile uint8_t g_debug_rf_ch = 0;
volatile uint8_t g_debug_config_after_switch = 0;
volatile uint8_t g_debug_ce_after_switch = 0;
volatile uint8_t g_debug_new_address_sent[5] = {0};
volatile uint8_t g_debug_ack_payload_checksum = 0;
volatile uint8_t g_debug_ack_sent_success = 0;
volatile uint8_t g_debug_ack_payload_full[16] = {0};
volatile uint8_t g_debug_rx_packet_count = 0;
volatile uint8_t g_debug_rx_checksum_error = 0;
volatile uint8_t g_debug_rx_format_error = 0;
volatile uint8_t g_debug_status_in_readdata = 0;
volatile uint8_t g_debug_senddata_status = 0;
volatile uint8_t g_debug_senddata_timeout = 0;

/* ============================================================================
 *                          SPI FUNCTIONS
 * ============================================================================ */

static uint8_t SPI_TxRx(uint8_t data)
{
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(&hspi3, &data, &rx_data, 1, NRF24L01_SPI_TIMEOUT);
    return rx_data;
}

static uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t value;
    NRF24L01_CSN_LOW();
    SPI_TxRx(reg);
    value = SPI_TxRx(NRF24L01_CMD_NOP);
    NRF24L01_CSN_HIGH();
    return value;
}

static uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    NRF24L01_CSN_LOW();
    status = SPI_TxRx(NRF24L01_CMD_WRITE_REG | reg);
    SPI_TxRx(value);
    NRF24L01_CSN_HIGH();
    return status;
}

static uint8_t NRF24L01_Read_To_Buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    NRF24L01_CSN_LOW();
    uint8_t status = SPI_TxRx(reg);
    while (len--) {
        *buf++ = SPI_TxRx(NRF24L01_CMD_NOP);
    }
    NRF24L01_CSN_HIGH();
    return status;
}

static uint8_t NRF24L01_Write_From_Buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    NRF24L01_CSN_LOW();
    uint8_t status = SPI_TxRx(reg);
    while (len--) {
        SPI_TxRx(*buf++);
    }
    NRF24L01_CSN_HIGH();
    return status;
}

static void NRF24L01_FlushRX(void)
{
    NRF24L01_Write_Reg(0xE2, NRF24L01_CMD_NOP);  // FLUSH_RX
}

static void NRF24L01_FlushTX(void)
{
    NRF24L01_Write_Reg(0xE1, NRF24L01_CMD_NOP);  // FLUSH_TX
}

static void NRF24L01_ClearIRQFlag(uint8_t flag)
{
    NRF24L01_Write_Reg(NRF24L01_REG_STATUS, flag);
}

/* ============================================================================
 *                          CHECKSUM
 * ============================================================================ */

static uint8_t hu_m40_check_sum(uint8_t *buf, uint8_t len)
{
    uint8_t i, sum = 0;
    for (i = 0; i < len; i++) {
        sum += buf[i];
    }
    return sum;
}

/* ============================================================================
 *                          MODE SWITCHING (exact copy from example)
 * ============================================================================ */

static void _NRF24L01_Config(uint8_t *tx_addr)
{
    NRF24L01_Write_From_Buf(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_TX_ADDR, tx_addr, 5);
    NRF24L01_Write_Reg(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_RX_PW_P0, PAYLOAD_WIDTH);
    NRF24L01_Write_Reg(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_EN_AA, 0x01);
    NRF24L01_Write_Reg(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_EN_RXADDR, 0x01);
    NRF24L01_Write_Reg(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_RF_CH, 25);
    NRF24L01_Write_Reg(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_RF_SETUP, 0x25);
    NRF24L01_Write_Reg(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_SETUP_RETR, 0x88);
}

static void NRF24L01_RX_Mode(uint8_t *rx_addr, uint8_t *tx_addr)
{
    NRF24L01_CE_LOW();
    _NRF24L01_Config(tx_addr);
    NRF24L01_Write_From_Buf(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_RX_ADDR_P0, rx_addr, 5);
    NRF24L01_Write_Reg(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_CONFIG, 0x0F);
    NRF24L01_CE_HIGH();
}

static void NRF24L01_TX_Mode(uint8_t *tx_addr)
{
    NRF24L01_CE_LOW();
    _NRF24L01_Config(tx_addr);
    NRF24L01_Write_From_Buf(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_RX_ADDR_P0, tx_addr, 5);
    NRF24L01_Write_Reg(NRF24L01_CMD_WRITE_REG | NRF24L01_REG_CONFIG, 0x0E);
    NRF24L01_CE_HIGH();
}

/* ============================================================================
 *                          SEND/RECEIVE DATA (exact copy from example)
 * ============================================================================ */

static void hu_m40_send_data(uint8_t *buf, uint8_t len)
{
    uint8_t status = 0x00;

    len = len > PAYLOAD_WIDTH ? PAYLOAD_WIDTH : len;
    NRF24L01_Write_From_Buf(0xA0, buf, len);  // TX payload

    while (1) {
        status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
        if (status & NRF24L01_STATUS_TX_DS) {
            NRF24L01_ClearIRQFlag(NRF24L01_STATUS_TX_DS);
            g_debug_ack_sent_success = 1;
            g_debug_senddata_status = status;
            break;
        } else if (status & NRF24L01_STATUS_MAX_RT) {
            NRF24L01_FlushTX();
            NRF24L01_ClearIRQFlag(NRF24L01_STATUS_MAX_RT);
            g_debug_ack_sent_success = 0;
            g_debug_senddata_status = status;
            break;
        }
    }
}

static uint8_t hu_m40_received_data(uint8_t *buf, uint8_t len)
{
    uint8_t status;

    status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
    if (status & NRF24L01_STATUS_RX_DR) {
        NRF24L01_Read_To_Buf(NRF24L01_CMD_RD_RX_PAYLOAD, buf, len);
        NRF24L01_ClearIRQFlag(NRF24L01_STATUS_RX_DR);
        return 1;
    }

    return 0;
}

/* ============================================================================
 *                          GPIO INITIALIZATION
 * ============================================================================ */

void NRF24L01_RX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = NRF24L01_CE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NRF24L01_CSN_PIN;
    HAL_GPIO_Init(NRF24L01_CSN_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NRF24L01_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NRF24L01_IRQ_PORT, &GPIO_InitStruct);

    NRF24L01_CE_LOW();
    NRF24L01_CSN_HIGH();
}

/* ============================================================================
 *                          INITIALIZATION (matching hu_m40_init)
 * ============================================================================ */

void NRF24L01_RX_Init(void)
{
    NRF24L01_RX_GPIO_Init();

    nrf_rx_handle.hspi = &hspi3;
    nrf_rx_handle.payload_width = PAYLOAD_WIDTH;

    HAL_Delay(100);

    /* Start in RX mode with default address */
    NRF24L01_RX_Mode((uint8_t *)HU_M40_ADDR_DEF, (uint8_t *)HU_M40_ADDR_DEF);
    g_hu_m40_tx_addr = (uint8_t *)HU_M40_ADDR_DEF;

    /* Initialize center values */
    g_hu_m40_rx_data[4] = 0x80;  // HU_RX
    g_hu_m40_rx_data[5] = 0x80;  // HU_RY
    g_hu_m40_rx_data[6] = 0x80;  // HU_LX
    g_hu_m40_rx_data[7] = 0x80;  // HU_LY

    nrf_rx_handle.is_initialized = 1;
    nrf_rx_handle.is_paired = 0;
    nrf_rx_handle.last_receive_time = 0;
}

/* ============================================================================
 *                          PAIRING
 * ============================================================================ */

uint8_t NRF24L01_RX_WaitForPairing(void)
{
    /* Just wait for first successful read */
    uint32_t start_time = HAL_GetTick();

    while ((HAL_GetTick() - start_time) < 10000) {
        if (NRF24L01_RX_ReadData()) {
            return 1;
        }
        HAL_Delay(1);
    }

    return 0;
}

uint8_t NRF24L01_RX_IsPaired(void)
{
    return nrf_rx_handle.is_paired;
}

uint8_t NRF24L01_RX_CheckData(void)
{
    uint8_t status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
    return (status & NRF24L01_STATUS_RX_DR) ? 1 : 0;
}

/* ============================================================================
 *                          DATA RECEPTION (exact copy of hu_m40_read)
 * ============================================================================ */

uint8_t NRF24L01_RX_ReadData(void)
{
    static uint32_t ticks = 0;
    uint16_t ticks1 = 500;
    uint8_t dat[PAYLOAD_WIDTH] = {0};
    static uint8_t res = 1;

    ticks++;

    if (ticks > 19999) {  // HU_M40_TIMEOUT_CNT
        ticks = 0;
        NRF24L01_RX_Init();
        nrf_rx_handle.is_paired = 0;
        res = 0;
    }

    while (ticks1--);  // Short delay

    if (hu_m40_received_data(dat, PAYLOAD_WIDTH)) {
        if (dat[0] == 1 && dat[1] == 3 && dat[15] == hu_m40_check_sum(dat, 15)) {
            ticks = 0;
            for (uint8_t i = 0; i < 16; i++) {
                g_hu_m40_rx_data[i] = dat[i];
            }

            /* Parse data */
            nrf_rx_handle.rolling_code = dat[2];
            nrf_rx_handle.rc_data.right_joystick_x = dat[4];
            nrf_rx_handle.rc_data.right_joystick_y = dat[5];
            nrf_rx_handle.rc_data.left_joystick_x = dat[6];
            nrf_rx_handle.rc_data.left_joystick_y = dat[7];
            nrf_rx_handle.rc_data.button_state = dat[8];
            nrf_rx_handle.rc_data.rolling_code = dat[2];
            nrf_rx_handle.rc_data.data_valid = 1;
            nrf_rx_handle.rc_data.timestamp = HAL_GetTick();
            nrf_rx_handle.last_receive_time = HAL_GetTick();

            /* Build response packet */
            dat[0] = 0x01;
            dat[1] = 0x83;
            dat[2] = dat[2];
            dat[3] = 11;
            dat[4] = g_hu_m40_rx_addr[0];
            dat[5] = g_hu_m40_rx_addr[1];
            dat[6] = g_hu_m40_rx_addr[2];
            dat[7] = g_hu_m40_rx_addr[3];
            dat[8] = g_hu_m40_rx_addr[4];
            dat[15] = hu_m40_check_sum(dat, 15);

            /* Debug */
            memcpy((void *)g_debug_ack_payload_full, dat, 16);
            memcpy((void *)g_debug_new_address_sent, g_hu_m40_rx_addr, 5);
            g_debug_ack_payload_checksum = dat[15];

            /* Send response */
            NRF24L01_TX_Mode(g_hu_m40_tx_addr);
            hu_m40_send_data(dat, PAYLOAD_WIDTH);

            /* Switch to RX mode with new address */
            NRF24L01_RX_Mode(g_hu_m40_rx_addr, g_hu_m40_rx_addr);
            g_hu_m40_tx_addr = g_hu_m40_rx_addr;

            /* Mark as paired */
            nrf_rx_handle.is_paired = 1;

            /* Debug tracking */
            g_debug_packets_on_new_addr++;
            g_debug_last_rolling_code = nrf_rx_handle.rolling_code;
            g_debug_observe_tx = NRF24L01_Read_Reg(0x08);
            g_debug_arc_cnt = g_debug_observe_tx & 0x0F;
            g_debug_ce_state = HAL_GPIO_ReadPin(NRF24L01_CE_PORT, NRF24L01_CE_PIN);

            static uint8_t local_packet_count = 0;
            if (local_packet_count == 0) {
                g_debug_first_packet[0] = dat[4];
                g_debug_first_packet[1] = dat[5];
                g_debug_first_packet[2] = dat[6];
                g_debug_first_packet[3] = dat[7];
                g_debug_en_aa = NRF24L01_Read_Reg(NRF24L01_REG_EN_AA);
                g_debug_en_rxaddr = NRF24L01_Read_Reg(NRF24L01_REG_EN_RXADDR);
                g_debug_rx_pw_p0 = NRF24L01_Read_Reg(NRF24L01_REG_RX_PW_P0);
                g_debug_feature = NRF24L01_Read_Reg(NRF24L01_REG_FEATURE);
                g_debug_config = NRF24L01_Read_Reg(NRF24L01_REG_CONFIG);
            }
            local_packet_count++;

            res = 1;
        }
    }

    return res;
}

RemoteControlData_t* NRF24L01_RX_GetData(void)
{
    return &nrf_rx_handle.rc_data;
}

uint8_t NRF24L01_RX_IsOnline(void)
{
    if (nrf_rx_handle.last_receive_time == 0) {
        return 0;
    }
    uint32_t elapsed = HAL_GetTick() - nrf_rx_handle.last_receive_time;
    return (elapsed < 500) ? 1 : 0;
}

uint32_t NRF24L01_RX_GetLastReceiveTime(void)
{
    return nrf_rx_handle.last_receive_time;
}

void NRF24L01_RX_SetAddress(uint8_t *address)
{
    /* Not used in this implementation */
    (void)address;
}

void NRF24L01_RX_FlushRx(void)
{
    NRF24L01_FlushRX();
}

uint8_t NRF24L01_RX_ReadRegister(uint8_t reg)
{
    return NRF24L01_Read_Reg(reg);
}

void NRF24L01_RX_WriteRegister(uint8_t reg, uint8_t value)
{
    NRF24L01_Write_Reg(reg, value);
}
