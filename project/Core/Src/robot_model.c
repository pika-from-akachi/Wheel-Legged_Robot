/**
  ******************************************************************************
  * @file    robot_model.c
  * @brief   Wheel-Legged Robot Kinematics and Dynamics Model Implementation
  ******************************************************************************
  */

#include "robot_model.h"
#include <math.h>
#include <string.h>

/* Default joint limits */
#define HIP_ANGLE_MIN       (-1.57f)   /**< -90 deg */
#define HIP_ANGLE_MAX       (1.57f)    /**<  90 deg */
#define KNEE_ANGLE_MIN      (-2.09f)   /**< -120 deg */
#define KNEE_ANGLE_MAX      (0.0f)     /**<   0 deg (straight) */
#define WHEEL_MAX_SPEED     (50.0f)    /**<  50 rad/s */

/* ============================================================================
 *                          LINEARIZED DYNAMICS MATRICES
 * ============================================================================
 *
 * Continuous-time LTI system: dx/dt = A*x + B*u
 *
 * For the inverted pendulum on wheels model:
 *   x = [body_angle, body_rate, wheel_pos, wheel_vel]^T
 *   u = [joint_torque, wheel_torque]^T
 *
 *   A = [0       1       0       0   ]
 *       [a21     0       0       0   ]
 *       [0       0       0       1   ]
 *       [a41     0       0       0   ]
 *
 *   B = [0       0   ]
 *       [b21     b22 ]
 *       [0       0   ]
 *       [b41     b42 ]
 *
 * These are placeholder values that should be tuned experimentally
 * or identified from the actual robot dynamics.
 * ============================================================================ */

void ROBOT_GetLinearizedDynamics(float A[4][4], float B[4][2], float standing_hip_angle)
{
    (void)standing_hip_angle;

    /* State matrix A (4x4) - placeholder linearized dynamics */
    /* These values approximate the inverted pendulum on wheels */
    memset(A, 0, 4 * 4 * sizeof(float));

    A[0][1] = 1.0f;     /* body_angle → body_rate coupling */
    A[1][0] = 85.0f;    /* gravitational effect on body acceleration */
    A[2][3] = 1.0f;     /* wheel_pos → wheel_vel coupling */
    A[3][0] = -12.0f;   /* body tilt affects wheel acceleration */

    /* Input matrix B (4x2) */
    memset(B, 0, 4 * 2 * sizeof(float));

    B[1][0] = 12.0f;    /* joint torque → body acceleration */
    B[1][1] = -3.0f;    /* wheel torque → body acceleration (reaction) */
    B[3][0] = -8.0f;    /* joint torque → wheel acceleration (reaction) */
    B[3][1] = 25.0f;    /* wheel torque → wheel acceleration */
}

/**
 * @brief Get LQR gain matrix K for balancing
 *
 * LQR cost: J = integral(x^T*Q*x + u^T*R*u) dt
 *
 * Q = diag(q1, q2, q3, q4)  — state weights
 *   q1: body angle weight     (high = stiff balancing)
 *   q2: body rate weight      (damping)
 *   q3: wheel position weight (position regulation)
 *   q4: wheel velocity weight (speed damping)
 *
 * R = diag(r1, r2)  — control effort weights
 *   r1: joint torque weight
 *   r2: wheel torque weight
 *
 * K is pre-computed offline for the linearized model.
 * These gains are starting points and should be tuned!
 */
void ROBOT_GetBalanceGains(float K[2][4], RobotMode_e mode)
{
    switch (mode) {
    case ROBOT_MODE_STANDING:
        K[0][0] = 0.0f;     K[0][1] = 0.0f;
        K[0][2] = 0.0f;     K[0][3] = 0.0f;
        K[1][0] = -0.5f;    /* body angle (P) */
        K[1][1] = -0.2f;    /* body rate (D) */
        K[1][2] = 0.0f;     /* wheel pos */
        K[1][3] = 0.0f;     /* wheel vel */
        break;

    case ROBOT_MODE_DRIVING:
        /* Softer balance during driving */
        K[0][0] = -15.81f;
        K[0][1] = -5.48f;
        K[0][2] = -1.58f;
        K[0][3] = -1.12f;
        K[1][0] = 3.54f;
        K[1][1] = 2.50f;
        K[1][2] = 1.12f;
        K[1][3] = 3.16f;
        break;

    case ROBOT_MODE_SITTING:
    case ROBOT_MODE_EMERGENCY_STOP:
    default:
        /* Zero gains — no active control */
        memset(K, 0, 2 * 4 * sizeof(float));
        break;
    }
}

/* ============================================================================
 *                          IMPLEMENTATION
 * ============================================================================ */

void ROBOT_Init(void)
{
    /* Nothing to initialize yet */
}

void ROBOT_ResetState(RobotState_t *state)
{
    memset(state, 0, sizeof(RobotState_t));
}

void ROBOT_SetJointPosition(RobotState_t *state, uint8_t motor_id, float position_rad)
{
    switch (motor_id) {
    case ROBOT_EL05_ID_HIP_L:  state->q_hip_L = position_rad; break;
    case ROBOT_EL05_ID_KNEE_L: state->q_knee_L = position_rad; break;
    case ROBOT_EL05_ID_HIP_R:  state->q_hip_R = position_rad; break;
    case ROBOT_EL05_ID_KNEE_R: state->q_knee_R = position_rad; break;
    default: break;
    }
}

void ROBOT_SetJointVelocity(RobotState_t *state, uint8_t motor_id, float velocity_rads)
{
    switch (motor_id) {
    case ROBOT_EL05_ID_HIP_L:  state->dq_hip_L = velocity_rads; break;
    case ROBOT_EL05_ID_KNEE_L: state->dq_knee_L = velocity_rads; break;
    case ROBOT_EL05_ID_HIP_R:  state->dq_hip_R = velocity_rads; break;
    case ROBOT_EL05_ID_KNEE_R: state->dq_knee_R = velocity_rads; break;
    default: break;
    }
}

float ROBOT_GetJointPosition(RobotState_t *state, uint8_t motor_id)
{
    switch (motor_id) {
    case ROBOT_EL05_ID_HIP_L:  return state->q_hip_L;
    case ROBOT_EL05_ID_KNEE_L: return state->q_knee_L;
    case ROBOT_EL05_ID_HIP_R:  return state->q_hip_R;
    case ROBOT_EL05_ID_KNEE_R: return state->q_knee_R;
    default: return 0.0f;
    }
}

float ROBOT_GetJointVelocity(RobotState_t *state, uint8_t motor_id)
{
    switch (motor_id) {
    case ROBOT_EL05_ID_HIP_L:  return state->dq_hip_L;
    case ROBOT_EL05_ID_KNEE_L: return state->dq_knee_L;
    case ROBOT_EL05_ID_HIP_R:  return state->dq_hip_R;
    case ROBOT_EL05_ID_KNEE_R: return state->dq_knee_R;
    default: return 0.0f;
    }
}

/**
 * @brief Extract reduced 4-dim balancing state from full 14-dim state
 */
BalanceState_t ROBOT_ExtractBalanceState(RobotState_t *full_state)
{
    BalanceState_t bal;
    bal.body_angle = full_state->q_body;
    bal.body_rate = full_state->dq_body;
    bal.wheel_position = (full_state->theta_wheel_L + full_state->theta_wheel_R) / 2.0f;
    bal.wheel_velocity = (full_state->dtheta_wheel_L + full_state->dtheta_wheel_R) / 2.0f;
    return bal;
}

/**
 * @brief Compute body height from ground (simplified 2D)
 */
float ROBOT_ComputeBodyHeight(RobotState_t *state)
{
    float hip_L = state->q_hip_L;
    float knee_L = state->q_knee_L;
    float hip_R = state->q_hip_R;
    float knee_R = state->q_knee_R;

    /* Simple average of both legs */
    float height_L = -ROBOT_UPPER_LEG_LENGTH * cosf(hip_L)
                     - ROBOT_LOWER_LEG_LENGTH * cosf(hip_L + knee_L);
    float height_R = -ROBOT_UPPER_LEG_LENGTH * cosf(hip_R)
                     - ROBOT_LOWER_LEG_LENGTH * cosf(hip_R + knee_R);

    return (height_L + height_R) / 2.0f + ROBOT_WHEEL_RADIUS;
}

/**
 * @brief Compute approximate center of gravity position
 */
float ROBOT_ComputeCOGPosition(RobotState_t *state)
{
    return ROBOT_ComputeBodyHeight(state) * 0.6f;
}

/**
 * @brief Clamp angle to [-pi, pi]
 */
float ROBOT_ClampAngle(float angle)
{
    while (angle > 3.14159f) angle -= 2.0f * 3.14159f;
    while (angle < -3.14159f) angle += 2.0f * 3.14159f;
    return angle;
}

/**
 * @brief Get default joint limits
 */
void ROBOT_GetDefaultJointLimits(JointLimits_t *limits)
{
    limits->hip_min = HIP_ANGLE_MIN;
    limits->hip_max = HIP_ANGLE_MAX;
    limits->knee_min = KNEE_ANGLE_MIN;
    limits->knee_max = KNEE_ANGLE_MAX;
    limits->wheel_max_speed = WHEEL_MAX_SPEED;
}
