/**
  ******************************************************************************
  * @file    m0601c_motor.c
  * @brief   M0601C Hub Motor Driver (RS485)
  * @note    RS485 half-duplex communication via USART1
  *          Protocol: 10-byte frames with CRC-8/MAXIM
  ******************************************************************************
  */

#include "m0601c_motor.h"
#include <string.h>

/* Private variables --------------------------------------------------------*/

static UART_HandleTypeDef *g_huart = NULL;
static GPIO_TypeDef *g_dir_port = NULL;
static uint16_t g_dir_pin = 0;

/* RS485 receive buffer (interrupt-driven byte collection) */
static volatile uint8_t g_rx_byte = 0;
static volatile uint8_t g_rx_buf[10];
static volatile uint8_t g_rx_idx = 0;
static volatile bool g_rx_complete = false;

/* CRC-8/MAXIM lookup table */
static const uint8_t crc8_table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

/* Private function prototypes ----------------------------------------------*/

static uint8_t M0601C_CRC8(uint8_t *data, uint16_t len);
static void M0601C_SetDirTX(void);
static void M0601C_SetDirRX(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Compute CRC-8/MAXIM (Dallas 1-Wire)
 * @param data Pointer to data buffer
 * @param len  Data length
 * @return CRC-8 value
 */
static uint8_t M0601C_CRC8(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

/**
 * @brief Set RS485 transceiver to transmit mode
 */
static void M0601C_SetDirTX(void)
{
    HAL_GPIO_WritePin(g_dir_port, g_dir_pin, GPIO_PIN_SET);
}

/**
 * @brief Set RS485 transceiver to receive mode
 */
static void M0601C_SetDirRX(void)
{
    HAL_GPIO_WritePin(g_dir_port, g_dir_pin, GPIO_PIN_RESET);
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize M0601C motor driver
 * @param huart    UART handle (USART1)
 * @param dir_port RS485 direction GPIO port
 * @param dir_pin  RS485 direction GPIO pin
 */
void M0601C_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *dir_port, uint16_t dir_pin)
{
    g_huart = huart;
    g_dir_port = dir_port;
    g_dir_pin = dir_pin;

    /* Default to receive mode */
    M0601C_SetDirRX();

    /* Clear RX buffer */
    g_rx_idx = 0;
    g_rx_complete = false;
    memset((void *)g_rx_buf, 0, sizeof(g_rx_buf));
}

/**
 * @brief Switch RS485 to receive mode
 */
void M0601C_RS485_EnterRx(void)
{
    M0601C_SetDirRX();
}

/**
 * @brief Switch RS485 to transmit mode
 */
void M0601C_RS485_EnterTx(void)
{
    M0601C_SetDirTX();
}

/**
 * @brief Transmit data over RS485
 * @param data Data buffer
 * @param len  Data length
 * @return HAL status
 */
HAL_StatusTypeDef M0601C_RS485_Transmit(uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    M0601C_SetDirTX();
    status = HAL_UART_Transmit(g_huart, data, len, M0601C_TX_TIMEOUT);
    M0601C_SetDirRX();

    return status;
}

/**
 * @brief Enable motor
 * @param motor Motor handle
 */
HAL_StatusTypeDef M0601C_Enable(M0601C_MotorHandle_t *motor)
{
    /* Command format: AA 55 o <id> 00 00 00 00 00 CRC */
    uint8_t cmd[10];
    cmd[0] = M0601C_FRAME_HEADER;
    cmd[1] = M0601C_FRAME_HEADER2;
    cmd[2] = M0601C_CMD_ENABLE;
    cmd[3] = motor->id;
    memset(&cmd[4], 0, 5);
    cmd[9] = M0601C_CRC8(cmd, 9);

    motor->state = M0601C_STATE_ENABLED;
    return M0601C_RS485_Transmit(cmd, 10);
}

/**
 * @brief Disable motor
 * @param motor Motor handle
 */
HAL_StatusTypeDef M0601C_Disable(M0601C_MotorHandle_t *motor)
{
    uint8_t cmd[10];
    cmd[0] = M0601C_FRAME_HEADER;
    cmd[1] = M0601C_FRAME_HEADER2;
    cmd[2] = M0601C_CMD_DISABLE;
    cmd[3] = motor->id;
    memset(&cmd[4], 0, 5);
    cmd[9] = M0601C_CRC8(cmd, 9);

    motor->state = M0601C_STATE_DISABLED;
    return M0601C_RS485_Transmit(cmd, 10);
}

/**
 * @brief Speed control command
 * @param motor     Motor handle
 * @param speed_rpm Target speed in RPM (-32768 to 32767)
 */
HAL_StatusTypeDef M0601C_SpeedControl(M0601C_MotorHandle_t *motor, int16_t speed_rpm)
{
    uint8_t cmd[10];
    cmd[0] = M0601C_FRAME_HEADER;
    cmd[1] = M0601C_FRAME_HEADER2;
    cmd[2] = M0601C_CMD_SPEED_CONTROL;
    cmd[3] = motor->id;
    cmd[4] = (uint8_t)(speed_rpm >> 8);   /* Speed high byte */
    cmd[5] = (uint8_t)(speed_rpm & 0xFF); /* Speed low byte */
    cmd[6] = 0x00;
    cmd[7] = 0x00;
    cmd[8] = 0x00;
    cmd[9] = M0601C_CRC8(cmd, 9);

    return M0601C_RS485_Transmit(cmd, 10);
}

/**
 * @brief Current control command
 * @param motor      Motor handle
 * @param current_ma Target current in mA (-8000 to 8000)
 */
HAL_StatusTypeDef M0601C_CurrentControl(M0601C_MotorHandle_t *motor, int16_t current_ma)
{
    uint8_t cmd[10];
    cmd[0] = M0601C_FRAME_HEADER;
    cmd[1] = M0601C_FRAME_HEADER2;
    cmd[2] = M0601C_CMD_CURRENT_CONTROL;
    cmd[3] = motor->id;
    cmd[4] = (uint8_t)(current_ma >> 8);   /* Current high byte */
    cmd[5] = (uint8_t)(current_ma & 0xFF); /* Current low byte */
    cmd[6] = 0x00;
    cmd[7] = 0x00;
    cmd[8] = 0x00;
    cmd[9] = M0601C_CRC8(cmd, 9);

    return M0601C_RS485_Transmit(cmd, 10);
}

/**
 * @brief Brake command
 * @param motor Motor handle
 */
HAL_StatusTypeDef M0601C_Brake(M0601C_MotorHandle_t *motor)
{
    uint8_t cmd[10];
    cmd[0] = M0601C_FRAME_HEADER;
    cmd[1] = M0601C_FRAME_HEADER2;
    cmd[2] = M0601C_CMD_BRAKE;
    cmd[3] = motor->id;
    memset(&cmd[4], 0, 5);
    cmd[9] = M0601C_CRC8(cmd, 9);

    return M0601C_RS485_Transmit(cmd, 10);
}

/**
 * @brief Idle (freewheel) command
 * @param motor Motor handle
 */
HAL_StatusTypeDef M0601C_Idle(M0601C_MotorHandle_t *motor)
{
    uint8_t cmd[10];
    cmd[0] = M0601C_FRAME_HEADER;
    cmd[1] = M0601C_FRAME_HEADER2;
    cmd[2] = M0601C_CMD_IDLE;
    cmd[3] = motor->id;
    memset(&cmd[4], 0, 5);
    cmd[9] = M0601C_CRC8(cmd, 9);

    return M0601C_RS485_Transmit(cmd, 10);
}

/**
 * @brief Request feedback from motor
 * @param motor Motor handle
 */
HAL_StatusTypeDef M0601C_RequestFeedback(M0601C_MotorHandle_t *motor)
{
    uint8_t cmd[10];
    cmd[0] = M0601C_FRAME_HEADER;
    cmd[1] = M0601C_FRAME_HEADER2;
    cmd[2] = M0601C_CMD_READ_FEEDBACK;
    cmd[3] = motor->id;
    memset(&cmd[4], 0, 5);
    cmd[9] = M0601C_CRC8(cmd, 9);

    return M0601C_RS485_Transmit(cmd, 10);
}

/**
 * @brief Set motor control mode
 * @param motor Motor handle
 * @param mode  Control mode
 */
HAL_StatusTypeDef M0601C_SetMode(M0601C_MotorHandle_t *motor, M0601C_Mode_e mode)
{
    uint8_t cmd[10];
    cmd[0] = M0601C_FRAME_HEADER;
    cmd[1] = M0601C_FRAME_HEADER2;
    cmd[2] = (uint8_t)mode;
    cmd[3] = motor->id;
    memset(&cmd[4], 0, 5);
    cmd[9] = M0601C_CRC8(cmd, 9);

    return M0601C_RS485_Transmit(cmd, 10);
}

/**
 * @brief Parse a 10-byte feedback frame and update motor handle
 * @param motor Motor handle to update
 * @param data  10-byte feedback frame (CRC already verified)
 * @return true if data was valid and matched motor ID
 */
bool M0601C_UpdateFeedback(M0601C_MotorHandle_t *motor, uint8_t *data)
{
    if (data[0] != motor->id) {
        return false;
    }

    motor->feedback.motor_id = data[0];
    motor->feedback.mode = data[1];

    /* Current: DATA[2:3] signed 16-bit, formula: mA = 8000 * raw / 32767 */
    int16_t current_raw = (int16_t)((data[2] << 8) | data[3]);
    motor->feedback.current_ma = (int16_t)((8000L * current_raw) / 32767);

    /* Speed: DATA[4:5] = RPM signed 16-bit */
    motor->feedback.speed_rpm = (int16_t)((data[4] << 8) | data[5]);

    /* Position: DATA[6:7] = unsigned 16-bit */
    motor->feedback.position_raw = (uint16_t)((data[6] << 8) | data[7]);

    /* Convert to angle: pos * 360 / 32768 */
    motor->feedback.position_deg = (float)motor->feedback.position_raw * 360.0f / 32768.0f;

    /* Temperature / status */
    motor->feedback.temperature = data[8];

    motor->feedback.is_valid = true;
    motor->feedback.timestamp = HAL_GetTick();
    motor->last_rx_tick = HAL_GetTick();
    motor->is_online = true;

    return true;
}

/**
 * @brief Get pointer to motor feedback data
 * @param motor Motor handle
 * @return Pointer to feedback structure
 */
M0601C_Feedback_t* M0601C_GetFeedback(M0601C_MotorHandle_t *motor)
{
    return &motor->feedback;
}

/**
 * @brief Check if motor is online (received data within timeout)
 * @param motor Motor handle
 * @return true if online
 */
bool M0601C_CheckOnline(M0601C_MotorHandle_t *motor)
{
    if (motor->is_online &&
        (HAL_GetTick() - motor->last_rx_tick < M0601C_ONLINE_TIMEOUT_MS)) {
        return true;
    }
    motor->is_online = false;
    return false;
}

/**
 * @brief RS485 UART RX callback (byte-by-byte collection)
 * @param byte Received byte
 * @note Called from USART1 IRQ handler
 */
void M0601C_RS485_RxCallback(uint8_t byte)
{
    if (g_rx_complete) {
        /* Frame already complete, waiting for processing */
        return;
    }

    /* Start of frame detection */
    if (g_rx_idx == 0 && byte != M0601C_FRAME_HEADER) {
        return; /* Not a valid start byte */
    }

    if (g_rx_idx == 1 && byte != M0601C_FRAME_HEADER2) {
        g_rx_idx = 0; /* Reset, invalid second header */
        return;
    }

    g_rx_buf[g_rx_idx++] = byte;

    /* Complete 10-byte frame received */
    if (g_rx_idx >= 10) {
        g_rx_idx = 0;
        g_rx_complete = true;
    }
}

/**
 * @brief Process complete RS485 frame (call from task context)
 * @param motor_handles Array of motor handles to check
 * @param num_motors    Number of motors in array
 * @return true if a valid frame was processed
 */
bool M0601C_ProcessRxFrame(M0601C_MotorHandle_t *motors, uint8_t num_motors)
{
    if (!g_rx_complete) {
        return false;
    }

    /* Verify CRC */
    uint8_t calc_crc = M0601C_CRC8((uint8_t *)g_rx_buf, 9);
    if (calc_crc != g_rx_buf[9]) {
        g_rx_complete = false;
        return false;
    }

    /* Find matching motor by ID and update */
    bool matched = false;
    for (uint8_t i = 0; i < num_motors; i++) {
        if (M0601C_UpdateFeedback(&motors[i], (uint8_t *)g_rx_buf)) {
            matched = true;
            break;
        }
    }

    g_rx_complete = false;
    return matched;
}
