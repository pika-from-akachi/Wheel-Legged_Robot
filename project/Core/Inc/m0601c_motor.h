/**
  ******************************************************************************
  * @file    m0601c_motor.h
  * @brief   M0601C Hub Motor Driver Header (RS485)
  * @note    Based on M0601C User Manual v1.0
  *          RS485 communication via USART1 with direction control
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

/* Exported types ------------------------------------------------------------*/

typedef enum {
    M0601C_MODE_SPEED = 0x01,     /**< Speed control mode */
    M0601C_MODE_CURRENT = 0x02,   /**< Current control mode */
    M0601C_MODE_POSITION = 0x03,  /**< Position control mode */
    M0601C_MODE_BRAKE = 0x04,     /**< Brake mode */
    M0601C_MODE_IDLE = 0x05       /**< Idle (freewheel) */
} M0601C_Mode_e;

typedef struct {
    uint8_t  motor_id;        /**< Motor ID (1-2 for this robot) */
    int16_t  current_ma;      /**< Current in mA */
    int16_t  speed_rpm;       /**< Speed in RPM */
    uint16_t position_raw;    /**< Raw position (0-65535) */
    float    position_deg;    /**< Position in degrees */
    uint8_t  temperature;     /**< Temperature in °C */
    uint8_t  mode;            /**< Current operating mode */
    bool     is_valid;        /**< Data validity flag */
    uint32_t timestamp;       /**< Timestamp of last update */
} M0601C_Feedback_t;

typedef enum {
    M0601C_STATE_DISABLED = 0,
    M0601C_STATE_ENABLED = 1,
    M0601C_STATE_ERROR = 2
} M0601C_State_e;

typedef struct {
    uint8_t         id;            /**< Motor CAN/RS485 ID (1-2) */
    M0601C_Mode_e   mode;          /**< Control mode */
    M0601C_State_e  state;         /**< Motor state */
    M0601C_Feedback_t feedback;    /**< Latest feedback data */
    uint32_t        last_rx_tick;  /**< Last received data tick */
    bool            is_online;     /**< Online status */
} M0601C_MotorHandle_t;

/* Command IDs */
#define M0601C_CMD_SPEED_CONTROL     0x64  /**< 'd' - Speed control */
#define M0601C_CMD_CURRENT_CONTROL   0x63  /**< 'c' - Current control */
#define M0601C_CMD_BRAKE             0x62  /**< 'b' - Brake */
#define M0601C_CMD_IDLE              0x69  /**< 'i' - Idle */
#define M0601C_CMD_READ_FEEDBACK     0x72  /**< 'r' - Read feedback */
#define M0601C_CMD_ENABLE            0x6F  /**< 'o' - Enable motor */
#define M0601C_CMD_DISABLE           0x70  /**< 'p' - Disable motor */

/* Protocol constants */
#define M0601C_FRAME_HEADER          0xAA
#define M0601C_FRAME_HEADER2         0x55
#define M0601C_FRAME_MIN_LEN         10
#define M0601C_RESPONSE_LEN          10
#define M0601C_TX_TIMEOUT            50
#define M0601C_RX_TIMEOUT            100
#define M0601C_ONLINE_TIMEOUT_MS     500

/* Exported functions --------------------------------------------------------*/

void M0601C_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *dir_port, uint16_t dir_pin);
HAL_StatusTypeDef M0601C_Enable(M0601C_MotorHandle_t *motor);
HAL_StatusTypeDef M0601C_Disable(M0601C_MotorHandle_t *motor);
HAL_StatusTypeDef M0601C_SpeedControl(M0601C_MotorHandle_t *motor, int16_t speed_rpm);
HAL_StatusTypeDef M0601C_CurrentControl(M0601C_MotorHandle_t *motor, int16_t current_ma);
HAL_StatusTypeDef M0601C_Brake(M0601C_MotorHandle_t *motor);
HAL_StatusTypeDef M0601C_Idle(M0601C_MotorHandle_t *motor);
HAL_StatusTypeDef M0601C_RequestFeedback(M0601C_MotorHandle_t *motor);
HAL_StatusTypeDef M0601C_SetMode(M0601C_MotorHandle_t *motor, M0601C_Mode_e mode);
M0601C_Feedback_t* M0601C_GetFeedback(M0601C_MotorHandle_t *motor);
bool M0601C_UpdateFeedback(M0601C_MotorHandle_t *motor, uint8_t *data);
bool M0601C_CheckOnline(M0601C_MotorHandle_t *motor);
void M0601C_RS485_RxCallback(uint8_t byte);

/* Shared RS485 interface */
void M0601C_RS485_EnterRx(void);
void M0601C_RS485_EnterTx(void);
HAL_StatusTypeDef M0601C_RS485_Transmit(uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __M0601C_MOTOR_H__ */
