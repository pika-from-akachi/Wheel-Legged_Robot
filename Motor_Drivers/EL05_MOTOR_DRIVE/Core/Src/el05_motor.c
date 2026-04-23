/**
  ******************************************************************************
  * @file    el05_motor.c
  * @brief   EL05 Motor Driver Implementation
  * @note    This file implements EL05 motor control functions
  *          Based on EL05 User Manual v1.0 (2025-11-25)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "el05_motor.h"
#include <string.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define CAN_EXT_ID_MASK     0x1FFFFFFFU
#define CAN_DLC_8           8

/* Private macro -------------------------------------------------------------*/
#define LIMIT(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* Private variables ---------------------------------------------------------*/
static CAN_HandleTypeDef *g_hcan = NULL;
static CAN_TxHeaderTypeDef g_tx_header;
static CAN_RxHeaderTypeDef g_rx_header;
static uint8_t g_tx_data[8] = {0};
static uint8_t g_rx_data[8] = {0};
static uint32_t g_tx_mailbox = 0;

/* Private function prototypes -----------------------------------------------*/
static uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits);
static HAL_StatusTypeDef EL05_SendCANFrame(uint32_t can_id, uint8_t *data, uint8_t dlc);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Convert float to unsigned integer with specified bit width
 * @param x: Input float value
 * @param x_min: Minimum value
 * @param x_max: Maximum value
 * @param bits: Bit width (8, 16, 32)
 * @retval Converted unsigned integer
 */
static uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    // Limit input value
    float x_sp = LIMIT(x, x_min, x_max);

    // Convert to integer
    return (uint32_t)((x_sp - x_min) / (x_max - x_min) * ((1 << bits) - 1));
}

/**
 * @brief Convert unsigned integer to float with specified bit width
 * @param x: Input unsigned integer
 * @param x_min: Minimum value
 * @param x_max: Maximum value
 * @param bits: Bit width (8, 16, 32)
 * @retval Converted float value
 */
static float uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits)
{
    return (float)(x) / ((1 << bits) - 1) * (x_max - x_min) + x_min;
}

/**
 * @brief Send CAN frame
 * @param can_id: CAN ID (extended frame)
 * @param data: Pointer to data buffer
 * @param dlc: Data length code
 * @retval HAL status
 */
static HAL_StatusTypeDef EL05_SendCANFrame(uint32_t can_id, uint8_t *data, uint8_t dlc)
{
    // Configure TX header
    g_tx_header.ExtId = can_id & CAN_EXT_ID_MASK;
    g_tx_header.IDE = CAN_ID_EXT;  // Extended ID
    g_tx_header.RTR = CAN_RTR_DATA; // Data frame
    g_tx_header.DLC = dlc;
    g_tx_header.TransmitGlobalTime = DISABLE;

    // Send CAN frame
    return HAL_CAN_AddTxMessage(g_hcan, &g_tx_header, data, &g_tx_mailbox);
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize EL05 motor driver
 * @param hcan: Pointer to CAN handle
 * @retval None
 */
void EL05_Init(CAN_HandleTypeDef *hcan)
{
    g_hcan = hcan;

    // Initialize TX/RX headers
    memset(&g_tx_header, 0, sizeof(CAN_TxHeaderTypeDef));
    memset(&g_rx_header, 0, sizeof(CAN_RxHeaderTypeDef));
    memset(g_tx_data, 0, 8);
    memset(g_rx_data, 0, 8);
}

/**
 * @brief Start CAN reception
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_StartReception(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    // Start CAN module
    status = HAL_CAN_Start(g_hcan);
    if (status != HAL_OK) {
        return status;
    }

    // Enable RX interrupt for FIFO 0
    status = HAL_CAN_ActivateNotification(g_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (status != HAL_OK) {
        return status;
    }

    // Enable RX interrupt for FIFO 1
    status = HAL_CAN_ActivateNotification(g_hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

/**
 * @brief Enable motor
 * @param motor: Pointer to motor handle
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_Enable(EL05_MotorHandle_t *motor)
{
    // CAN ID: 0x[Type][ID] = 0x0300 | ID
    uint32_t can_id = (EL05_TYPE_ENABLE << 8) | motor->can_id;

    // Send enable command
    HAL_StatusTypeDef status = EL05_SendCANFrame(can_id, g_tx_data, 0);

    if (status == HAL_OK) {
        motor->state = EL05_STATE_ENABLE;
    }

    return status;
}

/**
 * @brief Disable motor
 * @param motor: Pointer to motor handle
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_Disable(EL05_MotorHandle_t *motor)
{
    // CAN ID: 0x[Type][ID] = 0x0400 | ID
    uint32_t can_id = (EL05_TYPE_STOP << 8) | motor->can_id;

    // Send stop command
    HAL_StatusTypeDef status = EL05_SendCANFrame(can_id, g_tx_data, 0);

    if (status == HAL_OK) {
        motor->state = EL05_STATE_DISABLE;
    }

    return status;
}

/**
 * @brief Set motor control mode
 * @param motor: Pointer to motor handle
 * @param mode: Control mode
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_SetMode(EL05_MotorHandle_t *motor, EL05_ControlMode_e mode)
{
    // Write run_mode parameter (address 0x7005)
    HAL_StatusTypeDef status = EL05_WriteParam(motor, 0x7005, (float)mode);

    if (status == HAL_OK) {
        motor->mode = mode;
    }

    return status;
}

/**
 * @brief MIT mode control (运控模式)
 * @param motor: Pointer to motor handle
 * @param control: Pointer to control command
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_MitControl(EL05_MotorHandle_t *motor, EL05_MitControl_t *control)
{
    // CAN ID: 0x[Type][ID] = 0x0100 | ID
    uint32_t can_id = (EL05_TYPE_CONTROL << 8) | motor->can_id;

    // Limit control parameters
    float p_des = LIMIT(control->p_des, EL05_P_MIN, EL05_P_MAX);
    float v_des = LIMIT(control->v_des, EL05_V_MIN, EL05_V_MAX);
    float kp = LIMIT(control->kp, EL05_KP_MIN, EL05_KP_MAX);
    float kd = LIMIT(control->kd, EL05_KD_MIN, EL05_KD_MAX);
    float t_ff = LIMIT(control->t_ff, EL05_T_MIN, EL05_T_MAX);

    // Convert float to uint
    uint16_t p_uint = (uint16_t)float_to_uint(p_des, EL05_P_MIN, EL05_P_MAX, 16);
    uint16_t v_uint = (uint16_t)float_to_uint(v_des, EL05_V_MIN, EL05_V_MAX, 12);
    uint16_t kp_uint = (uint16_t)float_to_uint(kp, EL05_KP_MIN, EL05_KP_MAX, 12);
    uint16_t kd_uint = (uint16_t)float_to_uint(kd, EL05_KD_MIN, EL05_KD_MAX, 12);
    uint16_t t_uint = (uint16_t)float_to_uint(t_ff, EL05_T_MIN, EL05_T_MAX, 12);

    // Pack data into CAN frame (8 bytes)
    g_tx_data[0] = (p_uint >> 8) & 0xFF;  // p_des high byte
    g_tx_data[1] = p_uint & 0xFF;          // p_des low byte
    g_tx_data[2] = ((v_uint >> 4) & 0xFF); // v_des high 8 bits
    g_tx_data[3] = ((v_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF); // v_des low 4 bits + kp high 4 bits
    g_tx_data[4] = kp_uint & 0xFF;         // kp low 8 bits
    g_tx_data[5] = ((kd_uint >> 4) & 0xFF); // kd high 8 bits
    g_tx_data[6] = ((kd_uint & 0xF) << 4) | ((t_uint >> 8) & 0xF); // kd low 4 bits + t high 4 bits
    g_tx_data[7] = t_uint & 0xFF;          // t low 8 bits

    // Send CAN frame
    return EL05_SendCANFrame(can_id, g_tx_data, CAN_DLC_8);
}

/**
 * @brief Position control (CSP mode)
 * @param motor: Pointer to motor handle
 * @param position: Target position (rad)
 * @param velocity_limit: Velocity limit (rad/s)
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_PositionControl(EL05_MotorHandle_t *motor, float position, float velocity_limit)
{
    HAL_StatusTypeDef status;

    // Set mode to CSP if not already
    if (motor->mode != EL05_MODE_CSP) {
        status = EL05_SetMode(motor, EL05_MODE_CSP);
        if (status != HAL_OK) {
            return status;
        }
    }

    // Write limit_spd parameter (address 0x7017)
    status = EL05_WriteParam(motor, 0x7017, velocity_limit);
    if (status != HAL_OK) {
        return status;
    }

    // Write loc_ref parameter (address 0x7016)
    status = EL05_WriteParam(motor, 0x7016, position);

    return status;
}

/**
 * @brief Velocity control
 * @param motor: Pointer to motor handle
 * @param velocity: Target velocity (rad/s)
 * @param current_limit: Current limit (A)
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_VelocityControl(EL05_MotorHandle_t *motor, float velocity, float current_limit)
{
    HAL_StatusTypeDef status;

    // Set mode to velocity if not already
    if (motor->mode != EL05_MODE_VELOCITY) {
        status = EL05_SetMode(motor, EL05_MODE_VELOCITY);
        if (status != HAL_OK) {
            return status;
        }
    }

    // Write limit_cur parameter (address 0x2019)
    status = EL05_WriteParam(motor, 0x2019, current_limit);
    if (status != HAL_OK) {
        return status;
    }

    // Write spd_ref parameter (address 0x700A)
    status = EL05_WriteParam(motor, 0x700A, velocity);

    return status;
}

/**
 * @brief Current control
 * @param motor: Pointer to motor handle
 * @param current: Target current (A)
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_CurrentControl(EL05_MotorHandle_t *motor, float current)
{
    HAL_StatusTypeDef status;

    // Set mode to current if not already
    if (motor->mode != EL05_MODE_CURRENT) {
        status = EL05_SetMode(motor, EL05_MODE_CURRENT);
        if (status != HAL_OK) {
            return status;
        }
    }

    // Write iq_ref parameter (address 0x7006)
    status = EL05_WriteParam(motor, 0x7006, current);

    return status;
}

/**
 * @brief Set mechanical zero position
 * @param motor: Pointer to motor handle
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_SetZeroPosition(EL05_MotorHandle_t *motor)
{
    // CAN ID: 0x[Type][ID] = 0x0600 | ID
    uint32_t can_id = (EL05_TYPE_SET_ZERO << 8) | motor->can_id;

    // Send set zero command
    return EL05_SendCANFrame(can_id, g_tx_data, 0);
}

/**
 * @brief Write parameter
 * @param motor: Pointer to motor handle
 * @param param_addr: Parameter address
 * @param param_value: Parameter value
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_WriteParam(EL05_MotorHandle_t *motor, uint16_t param_addr, float param_value)
{
    // CAN ID: 0x[Type][ID] = 0x1200 | ID
    uint32_t can_id = (EL05_TYPE_WRITE_PARAM << 8) | motor->can_id;

    // Pack data into CAN frame
    g_tx_data[0] = (param_addr >> 8) & 0xFF;  // Address high byte
    g_tx_data[1] = param_addr & 0xFF;          // Address low byte

    // Convert float to bytes (IEEE 754)
    uint32_t value_uint = *(uint32_t*)&param_value;
    g_tx_data[2] = (value_uint >> 24) & 0xFF;
    g_tx_data[3] = (value_uint >> 16) & 0xFF;
    g_tx_data[4] = (value_uint >> 8) & 0xFF;
    g_tx_data[5] = value_uint & 0xFF;
    g_tx_data[6] = 0x00;  // Reserved
    g_tx_data[7] = 0x00;  // Reserved

    // Send CAN frame
    return EL05_SendCANFrame(can_id, g_tx_data, CAN_DLC_8);
}

/**
 * @brief Read parameter
 * @param motor: Pointer to motor handle
 * @param param_addr: Parameter address
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_ReadParam(EL05_MotorHandle_t *motor, uint16_t param_addr)
{
    // CAN ID: 0x[Type][ID] = 0x1100 | ID
    uint32_t can_id = (EL05_TYPE_READ_PARAM << 8) | motor->can_id;

    // Pack data into CAN frame
    g_tx_data[0] = (param_addr >> 8) & 0xFF;  // Address high byte
    g_tx_data[1] = param_addr & 0xFF;          // Address low byte
    g_tx_data[2] = 0x00;  // Reserved
    g_tx_data[3] = 0x00;
    g_tx_data[4] = 0x00;
    g_tx_data[5] = 0x00;
    g_tx_data[6] = 0x00;
    g_tx_data[7] = 0x00;

    // Send CAN frame
    return EL05_SendCANFrame(can_id, g_tx_data, CAN_DLC_8);
}

/**
 * @brief Get motor feedback data
 * @param motor: Pointer to motor handle
 * @retval Pointer to feedback data
 */
EL05_MotorFeedback_t* EL05_GetFeedback(EL05_MotorHandle_t *motor)
{
    return &motor->feedback;
}

/**
 * @brief Check motor online status
 * @param motor: Pointer to motor handle
 * @param timeout_ms: Timeout in milliseconds
 * @retval 1 if online, 0 if offline
 */
uint8_t EL05_CheckOnline(EL05_MotorHandle_t *motor, uint32_t timeout_ms)
{
    uint32_t current_time = HAL_GetTick();

    // Check if feedback was received within timeout
    if ((current_time - motor->last_update_time) < timeout_ms) {
        motor->is_online = 1;
        return 1;
    }

    motor->is_online = 0;
    return 0;
}

/**
 * @brief CAN RX callback function (called from HAL CAN interrupt)
 * @param hcan: Pointer to CAN handle
 * @retval None
 */
void EL05_CAN_RxCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // Get received message from FIFO 0
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        // Check if extended frame
        if (rx_header.IDE == CAN_ID_EXT)
        {
            uint32_t ext_id = rx_header.ExtId;

            // Parse CAN ID: [Type][ID]
            uint8_t msg_type = (ext_id >> 8) & 0xFF;
            uint8_t motor_id = ext_id & 0xFF;

            // Process based on message type
            if (msg_type == EL05_TYPE_FEEDBACK)
            {
                // Parse feedback data (Type 2)
                // Data format: [id][fault][position(2)][velocity(2)][torque(2)]
                // Note: This is simplified parsing, actual format may vary

                // For now, we'll store the raw data
                // In actual implementation, you need to parse according to the manual

                // Example parsing (adjust according to actual protocol):
                // uint8_t id = rx_data[0];
                // uint8_t fault = rx_data[1];
                // int16_t position_int = (rx_data[2] << 8) | rx_data[3];
                // int16_t velocity_int = (rx_data[4] << 8) | rx_data[5];
                // int16_t torque_int = (rx_data[6] << 8) | rx_data[7];

                // Convert to physical units
                // float position = uint_to_float(position_int, EL05_P_MIN, EL05_P_MAX, 16);
                // float velocity = uint_to_float(velocity_int, EL05_V_MIN, EL05_V_MAX, 12);
                // float torque = uint_to_float(torque_int, EL05_T_MIN, EL05_T_MAX, 12);

                // Update motor feedback structure
                // (This should be done by the user in their callback)
            }
        }
    }

    // Also check FIFO 1
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK)
    {
        // Same processing as FIFO 0
        if (rx_header.IDE == CAN_ID_EXT)
        {
            uint32_t ext_id = rx_header.ExtId;
            uint8_t msg_type = (ext_id >> 8) & 0xFF;
            uint8_t motor_id = ext_id & 0xFF;

            // Process message...
        }
    }
}
