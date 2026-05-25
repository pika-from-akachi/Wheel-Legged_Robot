/**
  ******************************************************************************
  * @file    lqr_control.h
  * @brief   LQR (Linear Quadratic Regulator) Control Algorithm Framework
  * @note    Implements state feedback control for wheel-legged robot balancing
  *
  *          Control law: u = -K * x + Kff * r
  *          where:  K  = LQR gain matrix (state feedback)
  *                  Kff = Feedforward gain matrix (reference tracking)
  *                  x  = state vector [body_angle, body_rate, wheel_pos, wheel_vel]
  *                  r  = reference vector
  *                  u  = control input [joint_torque, wheel_torque]
  ******************************************************************************
  */

#ifndef __LQR_CONTROL_H__
#define __LQR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "robot_model.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *                          LQR CONFIGURATION
 * ============================================================================ */

#define LQR_STATE_DIM       4   /**< Reduced state dimension */
#define LQR_CONTROL_DIM     2   /**< Reduced control dimension */
#define LQR_MAX_GAIN_SCHEDULES 4 /**< Max gain scheduling entries */

/* ============================================================================
 *                          DATA STRUCTURES
 * ============================================================================ */

/**
 * @brief LQR gain matrix K (control_dim x state_dim)
 *        u[control_dim] = -K[control_dim][state_dim] * x[state_dim]
 */
typedef struct {
    float K[LQR_CONTROL_DIM][LQR_STATE_DIM];  /**< State feedback gain */
    float Kff[LQR_CONTROL_DIM];                /**< Feedforward gain (for reference) */
} LQR_GainMatrix_t;

/**
 * @brief LQR controller state
 */
typedef struct {
    LQR_GainMatrix_t gain;          /**< Current gain matrix */
    float x_hat[LQR_STATE_DIM];     /**< Estimated state */
    float reference[LQR_STATE_DIM]; /**< Reference state */
    float u[LQR_CONTROL_DIM];       /**< Control output */
    RobotMode_e current_mode;       /**< Current robot mode */

    /* Integral action */
    float integral_error;           /**< Integral of body angle error */
    float integral_limit;           /**< Anti-windup limit */

    /* Saturation */
    float max_joint_torque;         /**< Max joint torque (N.m) */
    float max_wheel_torque;         /**< Max wheel torque (N.m) */

    /* Status */
    bool enabled;                   /**< Controller enabled */
    uint32_t control_count;         /**< Control loop counter */
    float loop_freq_hz;             /**< Control loop frequency (Hz) */
} LQR_Controller_t;

/**
 * @brief LQR tuning parameters (adjustable via ESP32)
 */
typedef struct {
    /* State cost weights (Q matrix diagonal) */
    float Q_body_angle;         /**< Body angle weight */
    float Q_body_rate;          /**< Body angular velocity weight */
    float Q_wheel_position;     /**< Wheel position weight */
    float Q_wheel_velocity;     /**< Wheel velocity weight */

    /* Control cost weights (R matrix diagonal) */
    float R_joint_torque;       /**< Joint torque effort weight */
    float R_wheel_torque;       /**< Wheel torque effort weight */

    /* Integral gain */
    float Ki_body_angle;        /**< Integral gain for body angle */

    /* Saturation limits */
    float max_joint_torque;     /**< Max joint torque (N.m) */
    float max_wheel_torque;     /**< Max wheel torque (N.m) */
    float max_integral;         /**< Integral anti-windup limit */
} LQR_TuningParams_t;

/* ============================================================================
 *                          DEFAULT PARAMETERS
 * ============================================================================ */

#define LQR_DEFAULT_Q_BODY_ANGLE     100.0f
#define LQR_DEFAULT_Q_BODY_RATE      10.0f
#define LQR_DEFAULT_Q_WHEEL_POS      1.0f
#define LQR_DEFAULT_Q_WHEEL_VEL      1.0f
#define LQR_DEFAULT_R_JOINT_TORQUE   0.1f
#define LQR_DEFAULT_R_WHEEL_TORQUE   0.5f
#define LQR_DEFAULT_KI               0.5f
#define LQR_DEFAULT_MAX_JOINT_TORQUE 6.0f
#define LQR_DEFAULT_MAX_WHEEL_TORQUE 3.0f
#define LQR_DEFAULT_MAX_INTEGRAL     2.0f

/* ============================================================================
 *                          FUNCTION PROTOTYPES
 * ============================================================================ */

/* Initialization */
void LQR_Init(LQR_Controller_t *lqr);
void LQR_SetDefaultTuning(LQR_TuningParams_t *params);

/* Core control law */
void LQR_Update(LQR_Controller_t *lqr, BalanceState_t *measured_state, float dt);
void LQR_ComputeControl(LQR_Controller_t *lqr, float dt);

/* Gain management */
void LQR_ComputeGains(LQR_TuningParams_t *params, LQR_GainMatrix_t *gain);
void LQR_SetGain(LQR_Controller_t *lqr, LQR_GainMatrix_t *gain);
void LQR_SetReference(LQR_Controller_t *lqr, float body_angle_ref,
                       float wheel_vel_ref);

/* Mode management */
void LQR_SetMode(LQR_Controller_t *lqr, RobotMode_e mode);
RobotMode_e LQR_GetMode(LQR_Controller_t *lqr);

/* Enable/Disable */
void LQR_Enable(LQR_Controller_t *lqr);
void LQR_Disable(LQR_Controller_t *lqr);
bool LQR_IsEnabled(LQR_Controller_t *lqr);

/* Parameter update (for ESP32 remote tuning) */
void LQR_UpdateTuning(LQR_Controller_t *lqr, LQR_TuningParams_t *params);
LQR_TuningParams_t* LQR_GetTuningParams(LQR_Controller_t *lqr);

/* Safety */
void LQR_EmergencyStop(LQR_Controller_t *lqr);
bool LQR_CheckSafety(LQR_Controller_t *lqr, BalanceState_t *state);

/* Utility */
float LQR_GetControlOutput(LQR_Controller_t *lqr, uint8_t channel);
void LQR_Reset(LQR_Controller_t *lqr);

#ifdef __cplusplus
}
#endif

#endif /* __LQR_CONTROL_H__ */
