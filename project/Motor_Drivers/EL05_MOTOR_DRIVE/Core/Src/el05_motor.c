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

/* Debug: last received motor ID from CAN response (Bit15~8 of ExtId) */
volatile uint8_t g_debug_rx_motor_id = 0;

/* Debug: CAN RX callback chain trace (Keil Watch查看) */
volatile uint32_t g_dbg_cb_fired   = 0;
volatile uint32_t g_dbg_rx_msg_ok  = 0;
volatile uint32_t g_dbg_rx_ext     = 0;
volatile uint32_t g_dbg_rx_mode2   = 0;
volatile uint32_t g_dbg_rx_matched = 0;
volatile uint32_t g_dbg_rx_ide     = 0;
volatile uint32_t g_dbg_rx_mode    = 0;
volatile uint32_t g_dbg_rx_mid     = 0;

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

    // Parameter address: low byte first (RS01 compatible)
    data[0] = param_addr & 0xFF;
    data[1] = (param_addr >> 8) & 0xFF;

    // Bytes 2-3: padding (RS01 protocol expectation)
    data[2] = 0x00;
    data[3] = 0x00;

    // Parameter value: float in little-endian at bytes 4-7
    memcpy(&data[4], &param_value, 4);

    return EL05_SendCANFrame(ext_id, data, 8);
}

/**
 * @brief Write uint8 parameter (RS01 'j' mode — single byte at offset 4)
 */
HAL_StatusTypeDef EL05_WriteParamU8(EL05_MotorHandle_t *motor, uint16_t param_addr, uint8_t param_value)
{
    uint32_t ext_id = EL05_BuildExtId(18, EL05_MASTER_ID, motor->can_id);
    uint8_t data[8] = {0};

    data[0] = param_addr & 0xFF;
    data[1] = (param_addr >> 8) & 0xFF;
    data[4] = param_value;

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
 * @brief Set current position as zero (通信类型6)
 */
HAL_StatusTypeDef EL05_SetZeroPosition(EL05_MotorHandle_t *motor)
{
    uint32_t ext_id = EL05_BuildExtId(6, EL05_MASTER_ID, motor->can_id);
    uint8_t data[8] = {0};
    return EL05_SendCANFrame(ext_id, data, 8);
}

/**
 * @brief Set motor CAN ID over the bus (通信类型7)
 * @note  参照例程 SampleProgram 的 Set_CAN_ID 实现。
 *        ExtID: mode=7, data2=(new_id<<8)|master_id, id=motor->can_id
 *        Data: 8 bytes all 0x00
 *        发送后需更新本地 motor->can_id 以匹配电机新 ID。
 */
HAL_StatusTypeDef EL05_SetMotorId(EL05_MotorHandle_t *motor, uint8_t new_id)
{
    uint32_t ext_id = EL05_BuildExtId(7,
                        (uint16_t)(((uint16_t)new_id << 8) | EL05_MASTER_ID),
                        motor->can_id);
    uint8_t data[8] = {0};
    return EL05_SendCANFrame(ext_id, data, 8);
}

/**
 * @brief Set motor protocol mode (通信类型0x19)
 * @note  必须在Enable之前调用
 *        mode_type = 0x01 → MIT模式
 */
HAL_StatusTypeDef EL05_SetMotorType(EL05_MotorHandle_t *motor, uint8_t mode_type)
{
    uint32_t ext_id = EL05_BuildExtId(0x19, EL05_MASTER_ID, motor->can_id);
    uint8_t data[8] = {0};
    data[0] = mode_type;
    return EL05_SendCANFrame(ext_id, data, 8);
}

/**
 * @brief Save motor parameters to NVRAM (通信类型0x16)
 * @note  参数掉电不丢失。调用后需等待1秒再操作
 */
HAL_StatusTypeDef EL05_MotorDataSave(EL05_MotorHandle_t *motor)
{
    uint32_t ext_id = EL05_BuildExtId(0x16, EL05_MASTER_ID, motor->can_id);
    uint8_t data[8] = {0};
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
    g_dbg_rx_msg_ok++;

    // Capture EVERY received frame for debugging (regardless of type)
    extern volatile uint32_t g_can_rx_raw_id;
    extern volatile uint8_t g_can_rx_raw_ide;
    extern volatile uint8_t g_can_rx_raw_dlc;
    extern volatile uint8_t g_can_rx_raw_data0;
    g_can_rx_raw_id = (rx_header.IDE == CAN_ID_EXT) ? rx_header.ExtId : rx_header.StdId;
    g_can_rx_raw_ide = (uint8_t)rx_header.IDE;
    g_can_rx_raw_dlc = rx_header.DLC;
    g_can_rx_raw_data0 = rx_data[0];

    // Debug: capture motor ID from EVERY extended frame (Bit15~8)
    extern volatile uint8_t g_debug_rx_motor_id;
    g_debug_rx_motor_id = (rx_header.ExtId >> 8) & 0xFF;

    // Only process extended frames
    g_dbg_rx_ide = rx_header.IDE;
    if (rx_header.IDE != CAN_ID_EXT) {
        return;
    }
    g_dbg_rx_ext++;

    // Extract communication type from Bit28~24
    uint8_t mode = (rx_header.ExtId >> 24) & 0x1F;
    g_dbg_rx_mode = mode;

    // Only process motor response (type 2)
    if (mode != 2) {
        return;
    }
    g_dbg_rx_mode2++;

    // Extract motor ID from Bit15~8 (RS01 protocol: response ID is in upper byte)
    uint8_t motor_id = (rx_header.ExtId >> 8) & 0xFF;
    g_dbg_rx_mid = motor_id;

    // Find motor handle by can_id (search global array)
    extern EL05_MotorHandle_t g_el05_motors[4];
    EL05_MotorHandle_t *motor = NULL;
    for (int i = 0; i < 4; i++) {
        if (g_el05_motors[i].can_id == motor_id) {
            motor = &g_el05_motors[i];
            break;
        }
    }
    if (motor == NULL) {
        return;
    }
    g_dbg_rx_matched++;

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

    // Capture all 8 raw bytes for debug
    extern volatile uint8_t g_can_rx_raw[8];
    for (int i = 0; i < 8; i++) g_can_rx_raw[i] = rx_data[i];

    // Update online status
    motor->last_update_time = HAL_GetTick();
    motor->is_online = 1;

    // Update state based on fault
    if (motor->feedback.fault != 0) {
        motor->state = EL05_STATE_ERROR;
    }

    // Capture raw feedback into global debug variables
    extern volatile int16_t g_motor_fb_pos_int;
    extern volatile int16_t g_motor_fb_vel_int;
    extern volatile int16_t g_motor_fb_trq_int;
    extern volatile uint16_t g_motor_fb_temp_int;
    extern volatile uint8_t  g_motor_fb_fault;
    extern volatile uint8_t  g_motor_fb_id;
    extern volatile uint8_t  g_motor_fb_mode_state;
    g_motor_fb_pos_int = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    g_motor_fb_vel_int = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    g_motor_fb_trq_int = (int16_t)((rx_data[6] << 8) | rx_data[7]);
    g_motor_fb_temp_int = (rx_header.ExtId >> 8) & 0xFFFF;
    g_motor_fb_fault = rx_data[1];
    g_motor_fb_id = (rx_data[0] >> 4) & 0x0F;
    g_motor_fb_mode_state = rx_data[0] & 0x0F;
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
