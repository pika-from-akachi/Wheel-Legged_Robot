/**
  ******************************************************************************
  * @file    esp32_com.c
  * @brief   ESP32-S3 Communication Protocol Implementation (UART3)
  ******************************************************************************
  */

#include "esp32_com.h"
#include <string.h>

/* Private variables --------------------------------------------------------*/

static ESP32_COM_t g_esp32;

/* Private function prototypes ----------------------------------------------*/

static void ESP32_COM_SendRaw(uint8_t *data, uint16_t len);
static void ESP32_COM_HandleCommand(ESP32_Packet_t *pkt);

/* CRC-16 CCITT table */
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

/* ============================================================================
 *                          INITIALIZATION
 * ============================================================================ */

void ESP32_COM_Init(UART_HandleTypeDef *huart)
{
    memset(&g_esp32, 0, sizeof(ESP32_COM_t));

    g_esp32.huart = huart;
    g_esp32.rx_state = ESP32_RX_IDLE;
    g_esp32.packet_ready = false;
    g_esp32.initialized = true;
}

void ESP32_COM_StartRx(void)
{
    g_esp32.rx_state = ESP32_RX_IDLE;
    g_esp32.rx_index = 0;
    g_esp32.packet_ready = false;
}

/* ============================================================================
 *                          CRC-16 CCITT
 * ============================================================================ */

uint16_t ESP32_COM_CRC16(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

/* ============================================================================
 *                          RAW UART FUNCTIONS
 * ============================================================================ */

static void ESP32_COM_SendRaw(uint8_t *data, uint16_t len)
{
    if (g_esp32.huart == NULL) return;
    HAL_UART_Transmit(g_esp32.huart, data, len, 100);
}

/* ============================================================================
 *                          PACKET BUILDING / SENDING
 * ============================================================================ */

HAL_StatusTypeDef ESP32_COM_SendPacket(uint8_t type, uint8_t *data, uint8_t len)
{
    if (!g_esp32.initialized) return HAL_ERROR;
    if (len > ESP32_COM_MAX_PAYLOAD) return HAL_ERROR;

    uint8_t *buf = g_esp32.tx_buffer;
    uint16_t idx = 0;

    buf[idx++] = ESP32_COM_START_BYTE_1;
    buf[idx++] = ESP32_COM_START_BYTE_2;
    buf[idx++] = type;
    buf[idx++] = len;

    if (len > 0 && data != NULL) {
        memcpy(&buf[idx], data, len);
        idx += len;
    }

    /* CRC covers type + length + payload */
    uint16_t crc = ESP32_COM_CRC16(&buf[2], len + 2);
    buf[idx++] = (uint8_t)(crc >> 8);
    buf[idx++] = (uint8_t)(crc & 0xFF);

    ESP32_COM_SendRaw(buf, idx);
    g_esp32.packets_sent++;

    return HAL_OK;
}

HAL_StatusTypeDef ESP32_COM_SendAck(uint8_t error_code)
{
    return ESP32_COM_SendPacket(PKT_TYPE_ACK, &error_code, 1);
}

/**
 * @brief Send full robot state data (called from task context)
 */
HAL_StatusTypeDef ESP32_COM_SendStateData(void)
{
    /* Collect state from externs */
    extern volatile float g_imu_accel_x_g;
    extern volatile float g_imu_accel_y_g;
    extern volatile float g_imu_accel_z_g;
    extern volatile float g_imu_gyro_x_dps;
    extern volatile float g_imu_gyro_y_dps;
    extern volatile float g_imu_gyro_z_dps;

    /* Pack: imu(6*4=24) + joint_pos(4*4=16) + wheel_speed(2*4=8) + mode(1) */
    uint8_t data[64];
    uint16_t idx = 0;

    memcpy(&data[idx], (void*)&g_imu_accel_x_g, 4); idx += 4;
    memcpy(&data[idx], (void*)&g_imu_accel_y_g, 4); idx += 4;
    memcpy(&data[idx], (void*)&g_imu_accel_z_g, 4); idx += 4;
    memcpy(&data[idx], (void*)&g_imu_gyro_x_dps, 4); idx += 4;
    memcpy(&data[idx], (void*)&g_imu_gyro_y_dps, 4); idx += 4;
    memcpy(&data[idx], (void*)&g_imu_gyro_z_dps, 4); idx += 4;

    return ESP32_COM_SendPacket(PKT_TYPE_STATE_DATA, data, idx);
}

HAL_StatusTypeDef ESP32_COM_SendMotorFeedback(uint8_t motor_id)
{
    (void)motor_id;
    /* TODO: Pack actual motor feedback */
    uint8_t data[4];
    data[0] = motor_id;
    return ESP32_COM_SendPacket(PKT_TYPE_MOTOR_FB, data, 4);
}

/* ============================================================================
 *                          RX PROCESSING
 * ============================================================================ */

bool ESP32_COM_IsPacketReady(void)
{
    return g_esp32.packet_ready;
}

uint8_t ESP32_COM_GetPacketType(void)
{
    return g_esp32.rx_buffer[2];
}

uint8_t* ESP32_COM_GetPacketPayload(void)
{
    return (uint8_t*)&g_esp32.rx_buffer[ESP32_COM_HEADER_LEN];
}

uint8_t ESP32_COM_GetPacketLength(void)
{
    return g_esp32.rx_payload_length;
}

void ESP32_COM_ProcessPacket(void)
{
    ESP32_Packet_t pkt;
    pkt.start1 = g_esp32.rx_buffer[0];
    pkt.start2 = g_esp32.rx_buffer[1];
    pkt.type = g_esp32.rx_buffer[2];
    pkt.length = g_esp32.rx_buffer[3];
    if (pkt.length > 0) {
        memcpy(pkt.payload, (void*)&g_esp32.rx_buffer[4], pkt.length);
    }

    /* Verify CRC */
    uint16_t calc_crc = ESP32_COM_CRC16(&g_esp32.rx_buffer[2], pkt.length + 2);
    uint16_t recv_crc = ((uint16_t)g_esp32.rx_buffer[4 + pkt.length] << 8) |
                         (uint16_t)g_esp32.rx_buffer[5 + pkt.length];

    if (calc_crc != recv_crc) {
        g_esp32.crc_errors++;
        ESP32_COM_SendAck(ESP32_ERR_CRC);
        ESP32_COM_ResetRx();
        return;
    }

    g_esp32.packets_received++;
    ESP32_COM_HandleCommand(&pkt);
    ESP32_COM_ResetRx();
}

void ESP32_COM_ResetRx(void)
{
    g_esp32.rx_state = ESP32_RX_IDLE;
    g_esp32.rx_index = 0;
    g_esp32.packet_ready = false;
}

/* ============================================================================
 *                          COMMAND HANDLER
 * ============================================================================ */

static void ESP32_COM_HandleCommand(ESP32_Packet_t *pkt)
{
    switch (pkt->type) {
    case PKT_TYPE_HEARTBEAT:
        g_esp32.last_heartbeat = HAL_GetTick();
        g_esp32.esp32_online = true;
        ESP32_COM_SendAck(ESP32_ERR_NONE);
        break;

    case PKT_TYPE_GET_STATE:
        ESP32_COM_SendStateData();
        break;

    case PKT_TYPE_FW_VERSION: {
        uint8_t version[] = {'W', 'L', 'R', '-', 'v', '1', '.', '0', '.', '0'};
        ESP32_COM_SendPacket(PKT_TYPE_FW_VERSION, version, 10);
        break;
    }

    case PKT_TYPE_SET_ENABLE: {
        if (pkt->length >= 1) {
            extern void LQR_Enable(void*);
            extern void LQR_Disable(void*);
            if (pkt->payload[0]) {
                /* enable */
            } else {
                /* disable */
            }
            ESP32_COM_SendAck(ESP32_ERR_NONE);
        }
        break;
    }

    case PKT_TYPE_SET_TUNING: {
        /* TODO: Parse and apply LQR tuning params from payload */
        ESP32_COM_SendAck(ESP32_ERR_NONE);
        break;
    }

    case PKT_TYPE_SET_MODE: {
        /* TODO: Set robot control mode from payload */
        ESP32_COM_SendAck(ESP32_ERR_NONE);
        break;
    }

    default:
        ESP32_COM_SendAck(ESP32_ERR_UNKNOWN_CMD);
        break;
    }
}

/* ============================================================================
 *                          UART INTERRUPT HANDLER
 * ============================================================================ */

void ESP32_COM_UART_IRQHandler(uint8_t byte)
{
    switch (g_esp32.rx_state) {
    case ESP32_RX_IDLE:
        if (byte == ESP32_COM_START_BYTE_1) {
            g_esp32.rx_state = ESP32_RX_START1;
            g_esp32.rx_buffer[0] = byte;
            g_esp32.rx_index = 1;
        }
        break;

    case ESP32_RX_START1:
        if (byte == ESP32_COM_START_BYTE_2) {
            g_esp32.rx_state = ESP32_RX_START2;
            g_esp32.rx_buffer[1] = byte;
            g_esp32.rx_index = 2;
        } else {
            g_esp32.rx_state = ESP32_RX_IDLE;
            g_esp32.rx_index = 0;
        }
        break;

    case ESP32_RX_START2:
        g_esp32.rx_state = ESP32_RX_TYPE;
        g_esp32.rx_buffer[2] = byte;
        g_esp32.rx_index = 3;
        break;

    case ESP32_RX_TYPE:
        g_esp32.rx_state = ESP32_RX_LENGTH;
        g_esp32.rx_buffer[3] = byte;
        g_esp32.rx_index = 4;
        g_esp32.rx_payload_length = byte;
        if (byte == 0) {
            g_esp32.rx_state = ESP32_RX_CRC1;
        } else {
            g_esp32.rx_state = ESP32_RX_PAYLOAD;
        }
        break;

    case ESP32_RX_PAYLOAD:
        g_esp32.rx_buffer[g_esp32.rx_index++] = byte;
        if (g_esp32.rx_index >= (uint16_t)(4 + g_esp32.rx_payload_length)) {
            g_esp32.rx_state = ESP32_RX_CRC1;
        }
        break;

    case ESP32_RX_CRC1:
        g_esp32.rx_buffer[g_esp32.rx_index++] = byte;
        g_esp32.rx_state = ESP32_RX_CRC2;
        break;

    case ESP32_RX_CRC2:
        g_esp32.rx_buffer[g_esp32.rx_index++] = byte;
        g_esp32.rx_state = ESP32_RX_COMPLETE;
        g_esp32.packet_ready = true;
        break;

    default:
        g_esp32.rx_state = ESP32_RX_IDLE;
        g_esp32.rx_index = 0;
        break;
    }
}

/* ============================================================================
 *                          STATUS
 * ============================================================================ */

bool ESP32_COM_IsOnline(void)
{
    return g_esp32.esp32_online &&
           (HAL_GetTick() - g_esp32.last_heartbeat < 2000);
}

void ESP32_COM_Update(void)
{
    /* Check for timeout */
    if (g_esp32.esp32_online &&
        (HAL_GetTick() - g_esp32.last_heartbeat > 5000)) {
        g_esp32.esp32_online = false;
    }

    /* Process pending packets */
    if (g_esp32.packet_ready) {
        ESP32_COM_ProcessPacket();
    }
}
