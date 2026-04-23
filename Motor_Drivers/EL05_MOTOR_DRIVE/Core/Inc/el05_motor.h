/**
  ******************************************************************************
  * @file    el05_motor.h
  * @brief   EL05 Motor Driver Header File
  * @note    This file contains EL05 motor control functions and data structures
  *          Based on EL05 User Manual v1.0 (2025-11-25)
  ******************************************************************************
  */

#ifndef __EL05_MOTOR_H__
#define __EL05_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief EL05 Motor Control Modes
 */
typedef enum {
    EL05_MODE_MIT = 0,       // MIT Minchee mode (运控模式)
    EL05_MODE_PP = 1,        // Position Profile mode (位置模式)
    EL05_MODE_VELOCITY = 2,  // Velocity mode (速度模式)
    EL05_MODE_CURRENT = 3,   // Current mode (电流模式)
    EL05_MODE_CSP = 5        // Cyclic Synchronous Position mode (位置模式CSP)
} EL05_ControlMode_e;

/**
 * @brief EL05 Motor State
 */
typedef enum {
    EL05_STATE_DISABLE = 0,  // Motor disabled
    EL05_STATE_ENABLE = 1,   // Motor enabled
    EL05_STATE_ERROR = 2     // Motor error state
} EL05_MotorState_e;

/**
 * @brief EL05 CAN Protocol Types
 */
typedef enum {
    EL05_TYPE_GET_ID = 0,        // Get device ID
    EL05_TYPE_CONTROL = 1,       // MIT control command
    EL05_TYPE_FEEDBACK = 2,      // Motor feedback data
    EL05_TYPE_ENABLE = 3,        // Motor enable
    EL05_TYPE_STOP = 4,          // Motor stop
    EL05_TYPE_SET_ZERO = 6,      // Set mechanical zero position
    EL05_TYPE_SET_CAN_ID = 7,    // Set CAN ID
    EL05_TYPE_READ_PARAM = 17,   // Read single parameter
    EL05_TYPE_WRITE_PARAM = 18,  // Write single parameter (lost on power off)
    EL05_TYPE_FAULT_FEEDBACK = 21, // Fault feedback
    EL05_TYPE_SAVE_DATA = 22,    // Save data
    EL05_TYPE_SET_BAUDRATE = 23, // Set baud rate
    EL05_TYPE_AUTO_REPORT = 24,  // Auto report
    EL05_TYPE_SET_PROTOCOL = 25, // Set protocol
    EL05_TYPE_GET_VERSION = 26   // Get version number
} EL05_ProtocolType_e;

/**
 * @brief EL05 Motor Feedback Data Structure
 */
typedef struct {
    uint8_t  id;           // Motor ID
    uint8_t  fault;        // Fault status
    float    position;     // Position (rad), range: -4π ~ 4π
    float    velocity;     // Velocity (rad/s)
    float    torque;       // Torque (N·m)
    float    temperature;  // Temperature (℃)
    uint32_t timestamp;    // Timestamp (ms)
} EL05_MotorFeedback_t;

/**
 * @brief EL05 Motor Control Command Structure
 */
typedef struct {
    float p_des;    // Desired position (rad)
    float v_des;    // Desired velocity (rad/s)
    float kp;       // Position gain
    float kd;       // Velocity gain
    float t_ff;     // Feedforward torque (N·m)
} EL05_MitControl_t;

/**
 * @brief EL05 Motor Handle Structure
 */
typedef struct {
    uint8_t can_id;                  // CAN ID (0-255)
    EL05_ControlMode_e mode;         // Control mode
    EL05_MotorState_e state;         // Motor state
    EL05_MotorFeedback_t feedback;   // Feedback data
    uint32_t last_update_time;       // Last update timestamp
    uint8_t is_online;               // Online status flag
} EL05_MotorHandle_t;

/* Exported constants --------------------------------------------------------*/

// EL05 Motor Specifications
#define EL05_RATED_VOLTAGE         48.0f    // V
#define EL05_RATED_TORQUE          1.8f     // N·m
#define EL05_PEAK_TORQUE           6.0f     // N·m
#define EL05_NO_LOAD_SPEED         430.0f   // rpm
#define EL05_GEAR_RATIO            9.0f     // Reduction ratio

// MIT Mode Parameter Limits
#define EL05_P_MIN    (-12.5f)    // Position min (rad)
#define EL05_P_MAX    (12.5f)     // Position max (rad)
#define EL05_V_MIN    (-30.0f)    // Velocity min (rad/s)
#define EL05_V_MAX    (30.0f)     // Velocity max (rad/s)
#define EL05_KP_MIN   (0.0f)      // Kp min
#define EL05_KP_MAX   (500.0f)    // Kp max
#define EL05_KD_MIN   (0.0f)      // Kd min
#define EL05_KD_MAX   (5.0f)      // Kd max
#define EL05_T_MIN    (-18.0f)    // Torque min (N·m)
#define EL05_T_MAX    (18.0f)     // Torque max (N·m)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Initialize EL05 motor driver
 * @param hcan: Pointer to CAN handle
 * @retval None
 */
void EL05_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief Start CAN reception
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_StartReception(void);

/**
 * @brief Enable motor
 * @param motor: Pointer to motor handle
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_Enable(EL05_MotorHandle_t *motor);

/**
 * @brief Disable motor
 * @param motor: Pointer to motor handle
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_Disable(EL05_MotorHandle_t *motor);

/**
 * @brief Set motor control mode
 * @param motor: Pointer to motor handle
 * @param mode: Control mode
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_SetMode(EL05_MotorHandle_t *motor, EL05_ControlMode_e mode);

/**
 * @brief MIT mode control (运控模式)
 * @param motor: Pointer to motor handle
 * @param control: Pointer to control command
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_MitControl(EL05_MotorHandle_t *motor, EL05_MitControl_t *control);

/**
 * @brief Position control (CSP mode)
 * @param motor: Pointer to motor handle
 * @param position: Target position (rad)
 * @param velocity_limit: Velocity limit (rad/s)
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_PositionControl(EL05_MotorHandle_t *motor, float position, float velocity_limit);

/**
 * @brief Velocity control
 * @param motor: Pointer to motor handle
 * @param velocity: Target velocity (rad/s)
 * @param current_limit: Current limit (A)
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_VelocityControl(EL05_MotorHandle_t *motor, float velocity, float current_limit);

/**
 * @brief Current control
 * @param motor: Pointer to motor handle
 * @param current: Target current (A)
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_CurrentControl(EL05_MotorHandle_t *motor, float current);

/**
 * @brief Set mechanical zero position
 * @param motor: Pointer to motor handle
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_SetZeroPosition(EL05_MotorHandle_t *motor);

/**
 * @brief Write parameter
 * @param motor: Pointer to motor handle
 * @param param_addr: Parameter address
 * @param param_value: Parameter value
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_WriteParam(EL05_MotorHandle_t *motor, uint16_t param_addr, float param_value);

/**
 * @brief Read parameter
 * @param motor: Pointer to motor handle
 * @param param_addr: Parameter address
 * @retval HAL status
 */
HAL_StatusTypeDef EL05_ReadParam(EL05_MotorHandle_t *motor, uint16_t param_addr);

/**
 * @brief Get motor feedback data
 * @param motor: Pointer to motor handle
 * @retval Pointer to feedback data
 */
EL05_MotorFeedback_t* EL05_GetFeedback(EL05_MotorHandle_t *motor);

/**
 * @brief Check motor online status
 * @param motor: Pointer to motor handle
 * @param timeout_ms: Timeout in milliseconds
 * @retval 1 if online, 0 if offline
 */
uint8_t EL05_CheckOnline(EL05_MotorHandle_t *motor, uint32_t timeout_ms);

/**
 * @brief CAN RX callback function (called from HAL CAN interrupt)
 * @param hcan: Pointer to CAN handle
 * @retval None
 */
void EL05_CAN_RxCallback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* __EL05_MOTOR_H__ */
