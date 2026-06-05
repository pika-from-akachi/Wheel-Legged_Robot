/**
  ******************************************************************************
  * @file    el05_motor.h
  * @brief   EL05 Motor Driver Header File
  * @note    Based on EL05 User Manual v1.0 (2025-11-25)
  *          Private protocol (CAN 2.0 extended frame)
  ******************************************************************************
  */

#ifndef __EL05_MOTOR_H__
#define __EL05_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

typedef enum {
    EL05_MODE_MIT = 0,       // 运控模式
    EL05_MODE_PP = 1,        // 位置模式PP
    EL05_MODE_VELOCITY = 2,  // 速度模式
    EL05_MODE_CURRENT = 3,   // 电流模式
    EL05_MODE_CSP = 5        // 位置模式CSP
} EL05_ControlMode_e;

typedef enum {
    EL05_STATE_DISABLE = 0,
    EL05_STATE_ENABLE = 1,
    EL05_STATE_ERROR = 2
} EL05_MotorState_e;

/**
 * @brief EL05 CAN Extended Frame ID structure
 * @note  29-bit ID: Bit28~24=通信类型, Bit23~8=data2, Bit7~0=目标地址
 */
typedef struct {
    uint32_t id : 8;      // Bit7~0: 目标电机CAN_ID
    uint32_t data2 : 16;  // Bit23~8: 数据区2 (master_id or torque_uint)
    uint32_t mode : 5;    // Bit28~24: 通信类型
    uint32_t res : 3;     // Bit31~29: 保留
} EL05_CanExtId_t;

typedef struct {
    uint8_t  id;
    uint8_t  fault;
    float    position;     // rad, range: -12.57 ~ 12.57
    float    velocity;     // rad/s, range: -50 ~ 50
    float    torque;       // N.m, range: -6 ~ 6
    float    temperature;  // ℃
    uint8_t  mode_state;   // 0:Reset, 1:Cali, 2:Motor
    uint32_t timestamp;
} EL05_MotorFeedback_t;

typedef struct {
    float p_des;    // 目标位置 (rad)
    float v_des;    // 目标速度 (rad/s)
    float kp;       // 位置增益 (0~500)
    float kd;       // 速度增益 (0~5)
    float t_ff;     // 前馈力矩 (N.m, -6~6)
} EL05_MitControl_t;

typedef struct {
    uint8_t can_id;
    EL05_ControlMode_e mode;
    EL05_MotorState_e state;
    EL05_MotorFeedback_t feedback;
    uint32_t last_update_time;
    uint8_t is_online;
} EL05_MotorHandle_t;

/* Exported constants --------------------------------------------------------*/

// EL05 Motor Specs
#define EL05_RATED_VOLTAGE     48.0f
#define EL05_RATED_TORQUE      1.8f
#define EL05_PEAK_TORQUE       6.0f
#define EL05_NO_LOAD_SPEED     430.0f
#define EL05_GEAR_RATIO        9.0f

// MIT Mode Parameter Limits (per manual section 4.4.2)
#define EL05_P_MIN    (-12.57f)   // rad
#define EL05_P_MAX    (12.57f)    // rad
#define EL05_V_MIN    (-50.0f)    // rad/s
#define EL05_V_MAX    (50.0f)     // rad/s
#define EL05_KP_MIN   (0.0f)
#define EL05_KP_MAX   (500.0f)
#define EL05_KD_MIN   (0.0f)
#define EL05_KD_MAX   (5.0f)
#define EL05_T_MIN    (-6.0f)     // N.m
#define EL05_T_MAX    (6.0f)      // N.m

// Default master CAN ID
#define EL05_MASTER_ID  0xFD      // 主机CAN_ID

/* Exported functions prototypes ---------------------------------------------*/

uint32_t float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(uint32_t x, float x_min, float x_max, int bits);

void EL05_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef EL05_Enable(EL05_MotorHandle_t *motor);
HAL_StatusTypeDef EL05_Disable(EL05_MotorHandle_t *motor);
HAL_StatusTypeDef EL05_MitControl(EL05_MotorHandle_t *motor, EL05_MitControl_t *control);
HAL_StatusTypeDef EL05_SetMode(EL05_MotorHandle_t *motor, EL05_ControlMode_e mode);
HAL_StatusTypeDef EL05_WriteParam(EL05_MotorHandle_t *motor, uint16_t param_addr, float param_value);
HAL_StatusTypeDef EL05_WriteParamU8(EL05_MotorHandle_t *motor, uint16_t param_addr, uint8_t param_value);
HAL_StatusTypeDef EL05_ReadParam(EL05_MotorHandle_t *motor, uint16_t param_addr);
HAL_StatusTypeDef EL05_SetMotorId(EL05_MotorHandle_t *motor, uint8_t new_id);
HAL_StatusTypeDef EL05_SetMotorType(EL05_MotorHandle_t *motor, uint8_t mode_type);  /* 0x19 */
HAL_StatusTypeDef EL05_MotorDataSave(EL05_MotorHandle_t *motor);                      /* 0x16 */
HAL_StatusTypeDef EL05_SetZeroPosition(EL05_MotorHandle_t *motor);                    /* 0x06 */
EL05_MotorFeedback_t* EL05_GetFeedback(EL05_MotorHandle_t *motor);
uint8_t EL05_CheckOnline(EL05_MotorHandle_t *motor, uint32_t timeout_ms);
void EL05_CAN_RxCallback(CAN_HandleTypeDef *hcan);

/* Debug: last received motor ID from CAN response frame (Keil Watch窗口查看) */
extern volatile uint8_t g_debug_rx_motor_id;

#ifdef __cplusplus
}
#endif

#endif /* __EL05_MOTOR_H__ */
