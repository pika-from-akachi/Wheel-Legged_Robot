/**
  ******************************************************************************
  * @file    m0601c_motor.h
  * @brief   M0601C Hub Motor Driver Header (RS485)
  * @note    帧协议对齐 project/Motor_Drivers/M0601C_DRIVE 中已验证的规范：
  *          - 驱动指令: [ID] 0x64 [value_H] [value_L] 0 0 [accTime] [brake] 0 [CRC8]
  *          - 反馈请求: [ID] 0x74 [8字节0] [CRC8]
  *          - 模式切换: [ID] 0xA0 [8字节0] [mode]
  ******************************************************************************
  */

#ifndef __M0601C_MOTOR_H__
#define __M0601C_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 控制模式值（对齐原协议） */
#define M0601C_MODE_CURRENT    0x01U
#define M0601C_MODE_SPEED      0x02U
#define M0601C_MODE_POSITION   0x03U

/* 指令码（对齐原协议） */
#define M0601C_CMD_DRIVE       0x64U
#define M0601C_CMD_FEEDBACK    0x74U
#define M0601C_CMD_MODE_SWITCH 0xA0U

/* 刹车控制 */
#define M0601C_BRAKE_ENABLE    0xFFU
#define M0601C_BRAKE_DISABLE   0x00U
#define M0601C_ACC_DEFAULT     0x00U

/* 协议常量 */
#define M0601C_FRAME_LEN          10U
#define M0601C_TX_TIMEOUT         50U
#define M0601C_ONLINE_TIMEOUT_MS  500U

/* Exported types ------------------------------------------------------------*/

typedef struct {
    uint8_t  motor_id;
    int16_t  current_ma;
    int16_t  speed_rpm;
    uint16_t position_raw;
    float    position_deg;
    uint8_t  temperature;
    uint8_t  mode;
    bool     is_valid;
    uint32_t timestamp;
} M0601C_Feedback_t;

typedef enum {
    M0601C_STATE_DISABLED = 0,
    M0601C_STATE_ENABLED = 1,
    M0601C_STATE_ERROR = 2
} M0601C_State_e;

typedef struct {
    uint8_t         id;
    uint8_t         mode;          /* 0x01/0x02/0x03 */
    M0601C_State_e  state;
    M0601C_Feedback_t feedback;
    uint32_t        last_rx_tick;
    bool            is_online;
} M0601C_MotorHandle_t;

/* Exported functions --------------------------------------------------------*/

void M0601C_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *dir_port, uint16_t dir_pin);

/* 控制命令（底层都走 0x64 驱动指令） */
HAL_StatusTypeDef M0601C_SpeedControl(M0601C_MotorHandle_t *motor, int16_t speed_rpm);
HAL_StatusTypeDef M0601C_CurrentControl(M0601C_MotorHandle_t *motor, int16_t current_raw);
HAL_StatusTypeDef M0601C_Brake(M0601C_MotorHandle_t *motor);
HAL_StatusTypeDef M0601C_Idle(M0601C_MotorHandle_t *motor);

/* 反馈与状态 */
HAL_StatusTypeDef M0601C_RequestFeedback(M0601C_MotorHandle_t *motor);
HAL_StatusTypeDef M0601C_SetMode(M0601C_MotorHandle_t *motor, uint8_t mode);
HAL_StatusTypeDef M0601C_Enable(M0601C_MotorHandle_t *motor);
HAL_StatusTypeDef M0601C_Disable(M0601C_MotorHandle_t *motor);
M0601C_Feedback_t* M0601C_GetFeedback(M0601C_MotorHandle_t *motor);
bool M0601C_UpdateFeedback(M0601C_MotorHandle_t *motor, uint8_t *data);
bool M0601C_CheckOnline(M0601C_MotorHandle_t *motor);

/* RS485 接口 */
void M0601C_RS485_EnterRx(void);
void M0601C_RS485_EnterTx(void);
void M0601C_RS485_RxCallback(uint8_t byte);
bool M0601C_ProcessRxFrame(M0601C_MotorHandle_t *motors, uint8_t num_motors);

#ifdef __cplusplus
}
#endif

#endif /* __M0601C_MOTOR_H__ */
