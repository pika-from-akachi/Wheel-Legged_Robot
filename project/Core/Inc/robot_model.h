/**
  ******************************************************************************
  * @file    robot_model.h
  * @brief   Wheel-Legged Robot Kinematics and Dynamics Model
  * @note    Defines the state space representation for LQR control
  *
  *          Robot Configuration:
  *          - 4 EL05 joint motors (CAN, IDs 1-4): hip/knee joints
  *          - 2 M0601C hub motors (RS485, IDs 1-2): wheel motors
  *          - 2D planar model for balance (sagittal plane)
  ******************************************************************************
  */

#ifndef __ROBOT_MODEL_H__
#define __ROBOT_MODEL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *                          ROBOT PARAMETERS
 * ============================================================================ */

/* Physical parameters (to be calibrated with actual robot) */
#define ROBOT_MASS                  12.0f   /**< Total robot mass (kg) */
#define ROBOT_BODY_LENGTH           0.35f   /**< Body length (m) */
#define ROBOT_UPPER_LEG_LENGTH      0.20f   /**< Upper leg segment length (m) */
#define ROBOT_LOWER_LEG_LENGTH      0.20f   /**< Lower leg segment length (m) */
#define ROBOT_WHEEL_RADIUS          0.05f   /**< Wheel radius (m) */
#define ROBOT_BODY_INERTIA          0.15f   /**< Body moment of inertia (kg*m^2) */
#define ROBOT_GRAVITY               9.81f   /**< Gravitational acceleration (m/s^2) */

/* ============================================================================
 *                          STATE SPACE DEFINITIONS
 * ============================================================================
 *
 * For 2D planar balancing model (sagittal plane):
 *   State vector x = [q_body, q_hip_L, q_knee_L, q_hip_R, q_knee_R,
 *                     theta_wheel_L, theta_wheel_R,
 *                     dq_body, dq_hip_L, dq_knee_L, dq_hip_R, dq_knee_R,
 *                     dtheta_wheel_L, dtheta_wheel_R]^T
 *
 *   Full state dimension: 14
 *
 *   Control input u = [tau_hip_L, tau_knee_L, tau_hip_R, tau_knee_R,
 *                      tau_wheel_L, tau_wheel_R]^T
 *   Control dimension: 6
 *
 *   For reduced balancing model (stationary, symmetric):
 *   State vector x = [q_body, dq_body, theta_wheel, dtheta_wheel]^T
 *   Control u = [tau_joint, tau_wheel]^T
 * ============================================================================ */

/* State dimensions */
#define ROBOT_STATE_DIM_FULL        14  /**< Full state dimension */
#define ROBOT_STATE_DIM_REDUCED     4   /**< Reduced balancing state dimension */
#define ROBOT_CONTROL_DIM_FULL      6   /**< Full control dimension */
#define ROBOT_CONTROL_DIM_REDUCED   2   /**< Reduced control dimension */
#define ROBOT_NUM_JOINTS            4   /**< Number of joint motors (EL05) */
#define ROBOT_NUM_WHEELS            2   /**< Number of wheel motors (M0601C) */

/* Motor ID mapping */
#define ROBOT_EL05_ID_HIP_L         1   /**< Left hip joint motor CAN ID */
#define ROBOT_EL05_ID_KNEE_L        2   /**< Left knee joint motor CAN ID */
#define ROBOT_EL05_ID_HIP_R         3   /**< Right hip joint motor CAN ID */
#define ROBOT_EL05_ID_KNEE_R        4   /**< Right knee joint motor CAN ID */
#define ROBOT_M0601C_ID_WHEEL_L     1   /**< Left wheel motor RS485 ID */
#define ROBOT_M0601C_ID_WHEEL_R     2   /**< Right wheel motor RS485 ID */

/* ============================================================================
 *                          DATA STRUCTURES
 * ============================================================================ */

/**
 * @brief Full robot state (14-dim)
 */
typedef struct {
    /* Joint positions (rad) */
    float q_body;           /**< Body pitch angle (rad) */
    float q_hip_L;          /**< Left hip angle (rad) */
    float q_knee_L;         /**< Left knee angle (rad) */
    float q_hip_R;          /**< Right hip angle (rad) */
    float q_knee_R;         /**< Right knee angle (rad) */
    float theta_wheel_L;    /**< Left wheel angle (rad) */
    float theta_wheel_R;    /**< Right wheel angle (rad) */

    /* Joint velocities (rad/s) */
    float dq_body;          /**< Body angular velocity (rad/s) */
    float dq_hip_L;         /**< Left hip velocity (rad/s) */
    float dq_knee_L;        /**< Left knee velocity (rad/s) */
    float dq_hip_R;         /**< Right hip velocity (rad/s) */
    float dq_knee_R;        /**< Right knee velocity (rad/s) */
    float dtheta_wheel_L;   /**< Left wheel angular velocity (rad/s) */
    float dtheta_wheel_R;   /**< Right wheel angular velocity (rad/s) */

    uint32_t timestamp;     /**< Timestamp (ms) */
} RobotState_t;

/**
 * @brief Reduced balancing state (4-dim)
 *        x = [q_body, dq_body, theta_wheel, dtheta_wheel]
 */
typedef struct {
    float body_angle;       /**< Body pitch angle (rad) */
    float body_rate;        /**< Body angular velocity (rad/s) */
    float wheel_position;   /**< Average wheel position (rad) */
    float wheel_velocity;   /**< Average wheel velocity (rad/s) */
} BalanceState_t;

/**
 * @brief Control input vector (6-dim)
 */
typedef struct {
    float tau_hip_L;        /**< Left hip torque (N.m) */
    float tau_knee_L;       /**< Left knee torque (N.m) */
    float tau_hip_R;        /**< Right hip torque (N.m) */
    float tau_knee_R;       /**< Right knee torque (N.m) */
    float tau_wheel_L;      /**< Left wheel torque (N.m) */
    float tau_wheel_R;      /**< Right wheel torque (N.m) */
} ControlInput_t;

/**
 * @brief Robot joint limits
 */
typedef struct {
    float hip_min;          /**< Min hip angle (rad) */
    float hip_max;          /**< Max hip angle (rad) */
    float knee_min;         /**< Min knee angle (rad) */
    float knee_max;         /**< Max knee angle (rad) */
    float wheel_max_speed;  /**< Max wheel speed (rad/s) */
} JointLimits_t;

/**
 * @brief Robot mode enum
 */
typedef enum {
    ROBOT_MODE_STANDING = 0,    /**< Standing / balancing */
    ROBOT_MODE_DRIVING,         /**< Driving / moving */
    ROBOT_MODE_SITTING,         /**< Sitting / resting */
    ROBOT_MODE_JUMPING,         /**< Jumping */
    ROBOT_MODE_EMERGENCY_STOP,  /**< Emergency stop */
    ROBOT_MODE_CALIBRATION      /**< Calibration mode */
} RobotMode_e;

/* ============================================================================
 *                          FUNCTION PROTOTYPES
 * ============================================================================ */

/* Initialization */
void ROBOT_Init(void);

/* State management */
void ROBOT_ResetState(RobotState_t *state);
void ROBOT_SetJointPosition(RobotState_t *state, uint8_t motor_id, float position_rad);
void ROBOT_SetJointVelocity(RobotState_t *state, uint8_t motor_id, float velocity_rads);
float ROBOT_GetJointPosition(RobotState_t *state, uint8_t motor_id);
float ROBOT_GetJointVelocity(RobotState_t *state, uint8_t motor_id);

/* Balance state extraction */
BalanceState_t ROBOT_ExtractBalanceState(RobotState_t *full_state);

/* Kinematics */
float ROBOT_ComputeBodyHeight(RobotState_t *state);
float ROBOT_ComputeCOGPosition(RobotState_t *state);

/* State space matrices for LQR (reduced 4-dim model) */
void ROBOT_GetLinearizedDynamics(float A[4][4], float B[4][2], float standing_hip_angle);
void ROBOT_GetBalanceGains(float K[2][4], RobotMode_e mode);

/* Utility */
float ROBOT_ClampAngle(float angle);
void ROBOT_GetDefaultJointLimits(JointLimits_t *limits);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_MODEL_H__ */
