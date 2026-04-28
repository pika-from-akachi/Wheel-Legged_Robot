/**
  ******************************************************************************
  * @file    el05_motor.c
  * @brief   EL05 Motor Driver
  * @note    Based on EL05 User Manual v1.0 (2025-11-25)
  *          Private protocol (CAN 2.0 extended frame)
  *
  *          29-bit Extended Frame ID layout:
  *          Bit28~24: 通信类型 (mode)
  *          Bit23~8:  数据区2 (data2) - master_id or torque_uint
  *          Bit7~0:   目标电机CAN_ID (id)
  *
  *          MIT Control (Type 1):
  *          Byte0~1: p_des 16bit unsigned (high byte first)
  *          Byte2~3: v_des 16bit unsigned
  *          Byte4~5: kp    16bit unsigned
  *          Byte6~7: kd    16bit unsigned
  *          t_ff packed in ExtID Bit23~8 as 16bit unsigned
  *
  *          Motor Response (Type 2):
  *          Byte0: id + mode_state
  *          Byte1: fault
  *          Byte2~3: position 16bit unsigned
  *          Byte4~5: velocity 16bit unsigned
  *          Byte6~7: torque 16bit unsigned
  *          temperature in ExtID Bit23~8
  ******************************************************************************
  */

#include "el05_motor.h"
#include <string.h>

/* Private variables --------------------------------------------------------*/
static CAN_HandleTypeDef *g_hcan = NULL;
static uint8_t g_tx_data[8];

/* Private function prototypes ----------------------------------------------*/
static HAL_StatusTypeDef EL05_SendCANFrame(uint32_t ext_id, uint8_t *data, uint8_t dlc);
static uint32_t EL05_BuildExtId(uint8_t mode, uint16_t data2, uint8_t motor_id);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Build 29-bit CAN extended frame ID
 * @param mode: 通信类型 (5-bit, Bit28~24)
 * @param data2: 数据区2 (16-bit, Bit23~8)
 * @param motor_id: 目标电机CAN_ID (8-bit, Bit7~0)
 * @retval 29-bit extended ID
 */
static uint32_t EL05_BuildExtId(uint8_t mode, uint16_t data2, uint8_t motor_id)
{
    return ((uint32_t)(mode & 0x1F) << 24) |
           ((uint32_t)(data2 & 0xFFFF) << 8) |
           ((uint32_t)(motor_id & 0xFF));
}

/**
 * @brief Convert float to unsigned integer with specified bit width
 */
uint32_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint32_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief Convert unsigned integer to float with specified bit width
 */
float uint_to_float(uint32_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return (float)x * span / (float)((1 << bits) - 1) + x_min;
}

/**
 * @brief Send CAN extended frame with timeout
 */
static HAL_StatusTypeDef EL05_SendCANFrame(uint32_t ext_id, uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.ExtId = ext_id;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    // Wait for free TX mailbox with timeout (100ms)
    uint32_t start_tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(g_hcan) == 0) {
        if ((HAL_GetTick() - start_tick) > 100) {
            return HAL_TIMEOUT;  // Timeout waiting for free mailbox
        }
    }

    return HAL_CAN_AddTxMessage(g_hcan, &tx_header, data, &tx_mailbox);
}

/* Exported functions --------------------------------------------------------*/

void EL05_Init(CAN_HandleTypeDef *hcan)
{
    g_hcan = hcan;
    memset(g_tx_data, 0, sizeof(g_tx_data));
}

/**
 * @brief Enable motor (通信类型3)
 * @note  ExtID: mode=3, data2=master_id, id=motor_id
 *        Data: 8 bytes all 0x00
 */
HAL_StatusTypeDef EL05_Enable(EL05_MotorHandle_t *motor)
{
    uint32_t ext_id = EL05_BuildExtId(3, EL05_MASTER_ID, motor->can_id);
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    HAL_StatusTypeDef status = EL05_SendCANFrame(ext_id, data, 8);
    if (status == HAL_OK) {
        motor->state = EL05_STATE_ENABLE;
    }
    return status;
}

/**
 * @brief Disable motor (通信类型4)
 * @note  ExtID: mode=4, data2=master_id, id=motor_id
 *        Data: 8 bytes all 0x00
 */
HAL_StatusTypeDef EL05_Disable(EL05_MotorHandle_t *motor)
{
    uint32_t ext_id = EL05_BuildExtId(4, EL05_MASTER_ID, motor->can_id);
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    HAL_StatusTypeDef status = EL05_SendCANFrame(ext_id, data, 8);
    if (status == HAL_OK) {
        motor->state = EL05_STATE_DISABLE;
    }
    return status;
}

/**
 * @brief MIT mode control (通信类型1)
 * @note  ExtID: mode=1, data2=t_ff_uint16, id=motor_id
 *        Data: p_des(2B) + v_des(2B) + kp(2B) + kd(2B), high byte first
 */
HAL_StatusTypeDef EL05_MitControl(EL05_MotorHandle_t *motor, EL05_MitControl_t *control)
{
    // Clamp parameters to valid range
    float p_des = control->p_des;
    float v_des = control->v_des;
    float kp = control->kp;
    float kd = control->kd;
    float t_ff = control->t_ff;

    if (p_des < EL05_P_MIN) p_des = EL05_P_MIN;
    if (p_des > EL05_P_MAX) p_des = EL05_P_MAX;
    if (v_des < EL05_V_MIN) v_des = EL05_V_MIN;
    if (v_des > EL05_V_MAX) v_des = EL05_V_MAX;
    if (kp < EL05_KP_MIN) kp = EL05_KP_MIN;
    if (kp > EL05_KP_MAX) kp = EL05_KP_MAX;
    if (kd < EL05_KD_MIN) kd = EL05_KD_MIN;
    if (kd > EL05_KD_MAX) kd = EL05_KD_MAX;
    if (t_ff < EL05_T_MIN) t_ff = EL05_T_MIN;
    if (t_ff > EL05_T_MAX) t_ff = EL05_T_MAX;

    // Convert to 16-bit unsigned
    uint16_t p_uint = (uint16_t)float_to_uint(p_des, EL05_P_MIN, EL05_P_MAX, 16);
    uint16_t v_uint = (uint16_t)float_to_uint(v_des, EL05_V_MIN, EL05_V_MAX, 16);
    uint16_t kp_uint = (uint16_t)float_to_uint(kp, EL05_KP_MIN, EL05_KP_MAX, 16);
    uint16_t kd_uint = (uint16_t)float_to_uint(kd, EL05_KD_MIN, EL05_KD_MAX, 16);
    uint16_t t_uint = (uint16_t)float_to_uint(t_ff, EL05_T_MIN, EL05_T_MAX, 16);

    // Pack data: high byte first (big-endian)
    uint8_t data[8];
    data[0] = (p_uint >> 8) & 0xFF;  // p_des high byte
    data[1] = p_uint & 0xFF;          // p_des low byte
    data[2] = (v_uint >> 8) & 0xFF;  // v_des high byte
    data[3] = v_uint & 0xFF;          // v_des low byte
    data[4] = (kp_uint >> 8) & 0xFF; // kp high byte
    data[5] = kp_uint & 0xFF;         // kp low byte
    data[6] = (kd_uint >> 8) & 0xFF; // kd high byte
    data[7] = kd_uint & 0xFF;         // kd low byte

    // Build ExtID: mode=1, data2=t_ff_uint, id=motor_id
    uint32_t ext_id = EL05_BuildExtId(1, t_uint, motor->can_id);

    return EL05_SendCANFrame(ext_id, data, 8);
}

/**
 * @brief Set motor control mode (通信类型5)
 * @note  ExtID: mode=5, data2=master_id, id=motor_id
 *        Data: Byte0=mode_value
 */
HAL_StatusTypeDef EL05_SetMode(EL05_MotorHandle_t *motor, EL05_ControlMode_e mode)
{
    uint32_t ext_id = EL05_BuildExtId(5, EL05_MASTER_ID, motor->can_id);
    uint8_t data[8] = {0};
    data[0] = (uint8_t)mode;

    HAL_StatusTypeDef status = EL05_SendCANFrame(ext_id, data, 8);
    if (status == HAL_OK) {
        motor->mode = mode;
    }
    return status;
}

/**
 * @brief Write parameter to motor (通信类型18)
 * @note  ExtID: mode=18, data2=master_id, id=motor_id
 *        Data: Byte0~1=param_addr (low byte first), Byte2~5=param_value (float, low byte first)
 */
HAL_StatusTypeDef EL05_WriteParam(EL05_MotorHandle_t *motor, uint16_t param_addr, float param_value)
{
    uint32_t ext_id = EL05_BuildExtId(18, EL05_MASTER_ID, motor->can_id);
    uint8_t data[8] = {0};

    // Parameter address: low byte first
    data[0] = param_addr & 0xFF;
    data[1] = (param_addr >> 8) & 0xFF;

    // Parameter value: float in little-endian
    memcpy(&data[2], &param_value, 4);

    return EL05_SendCANFrame(ext_id, data, 8);
}

/**
 * @brief Read parameter from motor (通信类型17)
 * @note  ExtID: mode=17, data2=master_id, id=motor_id
 *        Data: Byte0~1=param_addr (low byte first)
 */
HAL_StatusTypeDef EL05_ReadParam(EL05_MotorHandle_t *motor, uint16_t param_addr)
{
    uint32_t ext_id = EL05_BuildExtId(17, EL05_MASTER_ID, motor->can_id);
    uint8_t data[8] = {0};

    data[0] = param_addr & 0xFF;
    data[1] = (param_addr >> 8) & 0xFF;

    return EL05_SendCANFrame(ext_id, data, 8);
}

/**
 * @brief Parse motor response (通信类型2)
 * @note  ExtID: Bit23~8=temperature_uint
 *        Data: Byte0=id(4bit)+mode_state(4bit), Byte1=fault,
 *              Byte2~3=position 16bit, Byte4~5=velocity 16bit, Byte6~7=torque 16bit
 */
void EL05_CAN_RxCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    // Only process extended frames
    if (rx_header.IDE != CAN_ID_EXT) {
        return;
    }

    // Extract communication type from Bit28~24
    uint8_t mode = (rx_header.ExtId >> 24) & 0x1F;

    // Only process motor response (type 2)
    if (mode != 2) {
        return;
    }

    // Extract motor ID from Bit7~0
    uint8_t motor_id = rx_header.ExtId & 0xFF;

    // Find motor handle by can_id (check extern motor1)
    extern EL05_MotorHandle_t motor1;
    EL05_MotorHandle_t *motor = &motor1;
    if (motor->can_id != motor_id) {
        return;
    }

    // Parse Byte0: id(4bit) + mode_state(4bit)
    motor->feedback.id = (rx_data[0] >> 4) & 0x0F;
    motor->feedback.mode_state = rx_data[0] & 0x0F;

    // Parse Byte1: fault
    motor->feedback.fault = rx_data[1];

    // Parse Byte2~3: position 16bit (high byte first)
    uint16_t pos_uint = ((uint16_t)rx_data[2] << 8) | rx_data[3];
    motor->feedback.position = uint_to_float(pos_uint, EL05_P_MIN, EL05_P_MAX, 16);

    // Parse Byte4~5: velocity 16bit (high byte first)
    uint16_t vel_uint = ((uint16_t)rx_data[4] << 8) | rx_data[5];
    motor->feedback.velocity = uint_to_float(vel_uint, EL05_V_MIN, EL05_V_MAX, 16);

    // Parse Byte6~7: torque 16bit (high byte first)
    uint16_t tor_uint = ((uint16_t)rx_data[6] << 8) | rx_data[7];
    motor->feedback.torque = uint_to_float(tor_uint, EL05_T_MIN, EL05_T_MAX, 16);

    // Parse temperature from ExtID Bit23~8
    uint16_t temp_uint = (rx_header.ExtId >> 8) & 0xFFFF;
    motor->feedback.temperature = uint_to_float(temp_uint, -40.0f, 200.0f, 16);

    // Update online status
    motor->last_update_time = HAL_GetTick();
    motor->is_online = 1;

    // Update state based on fault
    if (motor->feedback.fault != 0) {
        motor->state = EL05_STATE_ERROR;
    }
}

EL05_MotorFeedback_t* EL05_GetFeedback(EL05_MotorHandle_t *motor)
{
    return &motor->feedback;
}

uint8_t EL05_CheckOnline(EL05_MotorHandle_t *motor, uint32_t timeout_ms)
{
    if (motor->is_online && (HAL_GetTick() - motor->last_update_time < timeout_ms)) {
        return 1;
    }
    motor->is_online = 0;
    return 0;
}
