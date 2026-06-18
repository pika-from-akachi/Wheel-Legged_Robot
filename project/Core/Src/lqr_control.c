/**
  ******************************************************************************
  * @file    lqr_control.c
  * @brief   LQR Control Algorithm Implementation
  *
  *          Discrete-time LQR state feedback:
  *          u[k] = -K * x[k] + Ki * integral(error) + Kff * r[k]
  *
  *          The gain matrix K is computed offline for the linearized
  *          inverted-pendulum-on-wheels model and stored as constants.
  *          Online gain scheduling is supported for different operation modes.
  ******************************************************************************
  */

#include "lqr_control.h"
#include <string.h>
#include <math.h>

/**
 * @brief Initialize LQR controller with default values
 */
void LQR_Init(LQR_Controller_t *lqr)
{
    memset(lqr, 0, sizeof(LQR_Controller_t));

    lqr->enabled = false;
    lqr->current_mode = ROBOT_MODE_SITTING;
    lqr->integral_limit = LQR_DEFAULT_MAX_INTEGRAL;
    lqr->max_joint_torque = LQR_DEFAULT_MAX_JOINT_TORQUE;
    lqr->max_wheel_torque = LQR_DEFAULT_MAX_WHEEL_TORQUE;
    lqr->loop_freq_hz = 1000.0f;

    /* Set default balance gains */
    ROBOT_GetBalanceGains(lqr->gain.K, ROBOT_MODE_STANDING);
    memset(lqr->gain.Kff, 0, sizeof(lqr->gain.Kff));

    /* Default reference: upright, stationary */
    lqr->reference[0] = 0.0f; /* body_angle = 0 (upright) */
    lqr->reference[1] = 0.0f; /* body_rate = 0 */
    lqr->reference[2] = 0.0f; /* wheel_position = 0 */
    lqr->reference[3] = 0.0f; /* wheel_velocity = 0 */
}

/**
 * @brief Set default tuning parameters
 */
void LQR_SetDefaultTuning(LQR_TuningParams_t *params)
{
    params->Q_body_angle = LQR_DEFAULT_Q_BODY_ANGLE;
    params->Q_body_rate = LQR_DEFAULT_Q_BODY_RATE;
    params->Q_wheel_position = LQR_DEFAULT_Q_WHEEL_POS;
    params->Q_wheel_velocity = LQR_DEFAULT_Q_WHEEL_VEL;
    params->R_joint_torque = LQR_DEFAULT_R_JOINT_TORQUE;
    params->R_wheel_torque = LQR_DEFAULT_R_WHEEL_TORQUE;
    params->Ki_body_angle = LQR_DEFAULT_KI;
    params->max_joint_torque = LQR_DEFAULT_MAX_JOINT_TORQUE;
    params->max_wheel_torque = LQR_DEFAULT_MAX_WHEEL_TORQUE;
    params->max_integral = LQR_DEFAULT_MAX_INTEGRAL;
}

/**
 * @brief Compute LQR gains from Q/R weights (offline Riccati approximation)
 *
 * For actual deployment, pre-compute gains using MATLAB/Python dlqr() and
 * store as constants. This function provides an approximation for online
 * tuning exploration.
 *
 * K = (R + B^T * P * B)^{-1} * B^T * P * A
 * where P solves the DARE: P = Q + A^T * P * A - A^T * P * B * (R + B^T * P * B)^{-1} * B^T * P * A
 *
 * For now, scales the default gains by weight ratios.
 */
void LQR_ComputeGains(LQR_TuningParams_t *params, LQR_GainMatrix_t *gain)
{
    float A[4][4], B[4][2];
    float default_gain_scale;

    (void)A;
    (void)B;

    ROBOT_GetLinearizedDynamics(A, B, 0.0f);

    /* Scale default gains based on Q/R ratio (approximate) */
    float qr_ratio_joint = params->Q_body_angle / params->R_joint_torque;
    float qr_ratio_wheel = params->Q_wheel_velocity / params->R_wheel_torque;

    /* Get base gains for standing mode */
    ROBOT_GetBalanceGains(gain->K, ROBOT_MODE_STANDING);

    /* Scale gains by sqrt of relative Q/R ratio */
    default_gain_scale = sqrtf(qr_ratio_joint / (LQR_DEFAULT_Q_BODY_ANGLE / LQR_DEFAULT_R_JOINT_TORQUE));
    for (int i = 0; i < 4; i++) {
        gain->K[0][i] *= default_gain_scale;
    }

    default_gain_scale = sqrtf(qr_ratio_wheel / (LQR_DEFAULT_Q_WHEEL_VEL / LQR_DEFAULT_R_WHEEL_TORQUE));
    for (int i = 0; i < 4; i++) {
        gain->K[1][i] *= default_gain_scale;
    }

    /* Feedforward gains (typically zero for regulation) */
    gain->Kff[0] = 0.0f;
    gain->Kff[1] = 0.0f;
}

/**
 * @brief Set LQR gain matrix
 */
void LQR_SetGain(LQR_Controller_t *lqr, LQR_GainMatrix_t *gain)
{
    memcpy(&lqr->gain, gain, sizeof(LQR_GainMatrix_t));
}

/**
 * @brief Set reference state
 * @param body_angle_ref   Reference body angle (rad), 0 = upright
 * @param wheel_vel_ref    Reference wheel velocity (rad/s), for driving
 */
void LQR_SetReference(LQR_Controller_t *lqr, float body_angle_ref, float wheel_vel_ref)
{
    lqr->reference[0] = body_angle_ref;
    lqr->reference[1] = 0.0f;
    lqr->reference[2] = 0.0f;
    lqr->reference[3] = wheel_vel_ref;
}

/**
 * @brief Set robot operation mode (affects gain scheduling)
 */
void LQR_SetMode(LQR_Controller_t *lqr, RobotMode_e mode)
{
    if (lqr->current_mode == mode) return;

    lqr->current_mode = mode;

    /* Update gains for new mode */
    LQR_GainMatrix_t new_gain;
    ROBOT_GetBalanceGains(new_gain.K, mode);
    memset(new_gain.Kff, 0, sizeof(new_gain.Kff));
    LQR_SetGain(lqr, &new_gain);

    /* Reset integral on mode change */
    lqr->integral_error = 0.0f;
}

RobotMode_e LQR_GetMode(LQR_Controller_t *lqr)
{
    return lqr->current_mode;
}

/**
 * @brief Enable controller
 */
void LQR_Enable(LQR_Controller_t *lqr)
{
    lqr->enabled = true;
    lqr->integral_error = 0.0f;
    lqr->control_count = 0;
}

/**
 * @brief Disable controller (zero output)
 */
void LQR_Disable(LQR_Controller_t *lqr)
{
    lqr->enabled = false;
    lqr->u[0] = 0.0f;
    lqr->u[1] = 0.0f;
    lqr->integral_error = 0.0f;
}

bool LQR_IsEnabled(LQR_Controller_t *lqr)
{
    return lqr->enabled;
}

/**
 * @brief Update LQR controller with new measurement
 *
 * Control law:
 *   u = -K * (x_hat - reference) + Ki * integral(error)
 *
 * where:
 *   x_hat = measured_state (or estimated state)
 *   error = x_hat[0] - reference[0]  (body angle error for integral)
 */
void LQR_Update(LQR_Controller_t *lqr, BalanceState_t *measured_state, float dt)
{
    if (!lqr->enabled) {
        lqr->u[0] = 0.0f;
        lqr->u[1] = 0.0f;
        return;
    }

    /* Copy measured state to estimated state */
    lqr->x_hat[0] = measured_state->body_angle;
    lqr->x_hat[1] = measured_state->body_rate;
    lqr->x_hat[2] = measured_state->wheel_position;
    lqr->x_hat[3] = measured_state->wheel_velocity;

    /* Compute state error (for reference tracking) */
    float error[LQR_STATE_DIM];
    for (int i = 0; i < LQR_STATE_DIM; i++) {
        error[i] = lqr->x_hat[i] - lqr->reference[i];
    }

    /* Integral (only when near upright to prevent windup) */
    if (fabsf(error[0]) < 0.2f) {
        lqr->integral_error += error[0] * dt;
    }
    if (lqr->integral_error > lqr->integral_limit) {
        lqr->integral_error = lqr->integral_limit;
    } else if (lqr->integral_error < -lqr->integral_limit) {
        lqr->integral_error = -lqr->integral_limit;
    }

    /* Compute control: u = -K * error + Ki * integral */
    lqr->u[0] = -(lqr->gain.K[0][0] * error[0] +
                   lqr->gain.K[0][1] * error[1] +
                   lqr->gain.K[0][2] * error[2] +
                   lqr->gain.K[0][3] * error[3]);
    lqr->u[0] += lqr->gain.Kff[0] * lqr->reference[0];

    lqr->u[1] = -(lqr->gain.K[1][0] * error[0] +
                   lqr->gain.K[1][1] * error[1] +
                   lqr->gain.K[1][2] * error[2] +
                   lqr->gain.K[1][3] * error[3]);
    lqr->u[1] += lqr->gain.Kff[1] * lqr->reference[3];

    /* Apply integral (unused, joint channel) */
    lqr->u[1] -= lqr->integral_error * LQR_DEFAULT_KI;

    /* Saturate outputs */
    LQR_ComputeControl(lqr, dt);

    lqr->control_count++;
}

/**
 * @brief Saturate control outputs to safe limits
 */
void LQR_ComputeControl(LQR_Controller_t *lqr, float dt)
{
    (void)dt;

    /* Saturate joint torque (channel 0) */
    if (lqr->u[0] > lqr->max_joint_torque) {
        lqr->u[0] = lqr->max_joint_torque;
    } else if (lqr->u[0] < -lqr->max_joint_torque) {
        lqr->u[0] = -lqr->max_joint_torque;
    }

    /* Saturate wheel torque (channel 1) */
    if (lqr->u[1] > lqr->max_wheel_torque) {
        lqr->u[1] = lqr->max_wheel_torque;
    } else if (lqr->u[1] < -lqr->max_wheel_torque) {
        lqr->u[1] = -lqr->max_wheel_torque;
    }
}

/**
 * @brief Update tuning parameters and recompute gains
 */
void LQR_UpdateTuning(LQR_Controller_t *lqr, LQR_TuningParams_t *params)
{
    lqr->max_joint_torque = params->max_joint_torque;
    lqr->max_wheel_torque = params->max_wheel_torque;
    lqr->integral_limit = params->max_integral;

    /* Recompute gains from new Q/R weights */
    LQR_GainMatrix_t new_gain;
    LQR_ComputeGains(params, &new_gain);
    LQR_SetGain(lqr, &new_gain);
}

LQR_TuningParams_t* LQR_GetTuningParams(LQR_Controller_t *lqr)
{
    (void)lqr;
    static LQR_TuningParams_t params;
    LQR_SetDefaultTuning(&params);
    return &params;
}

/**
 * @brief Emergency stop - disable controller and zero outputs
 */
void LQR_EmergencyStop(LQR_Controller_t *lqr)
{
    LQR_Disable(lqr);
    lqr->current_mode = ROBOT_MODE_EMERGENCY_STOP;
}

/**
 * @brief Safety check - verify state is within safe bounds
 * @return true if safe, false if should trigger emergency stop
 */
bool LQR_CheckSafety(LQR_Controller_t *lqr, BalanceState_t *state)
{
    (void)lqr;

    /* Body angle exceeds safe limit (±30 deg) */
    if (fabsf(state->body_angle) > 0.52f) {
        return false;
    }

    /* Body rate too high */
    if (fabsf(state->body_rate) > 8.0f) {
        return false;
    }

    /* Wheel speed too high */
    if (fabsf(state->wheel_velocity) > 50.0f) {
        return false;
    }

    return true;
}

/**
 * @brief Get current control output for a specific channel
 * @param channel 0 = joint torque, 1 = wheel torque
 */
float LQR_GetControlOutput(LQR_Controller_t *lqr, uint8_t channel)
{
    if (channel >= LQR_CONTROL_DIM) return 0.0f;
    return lqr->u[channel];
}

/**
 * @brief Reset controller state
 */
void LQR_Reset(LQR_Controller_t *lqr)
{
    lqr->integral_error = 0.0f;
    lqr->u[0] = 0.0f;
    lqr->u[1] = 0.0f;
    lqr->control_count = 0;
}
