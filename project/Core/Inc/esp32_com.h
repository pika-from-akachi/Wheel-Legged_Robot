/**
  ******************************************************************************
  * @file    esp32_com.h
  * @brief   ESP32-S3 Communication Protocol for STM32F407 (UART3)
  * @note    Half-duplex packet protocol over UART3 (PB10=TX, PB11=RX)
  *
  *          Protocol:
  *          - Start bytes: 0xAA 0xBB
  *          - Packet type: 1 byte (command/response)
  *          - Data length: 1 byte
  *          - Payload: variable length
  *          - CRC-16: 2 bytes (CCITT)
  *
  *          Command types:
  *          0x01: Set LQR tuning params
  *          0x02: Get robot state
  *          0x03: Set control mode
  *          0x04: Enable/Disable motors
  *          0x05: Set motor position/speed
  *          0x06: Heartbeat / Ping
  *          0x10-0x1F: Motor-specific commands
  *          0x20: System reset
  *          0x30: Firmware version
  *          0xF0: Raw debug data stream
  ******************************************************************************
  */

#ifndef __ESP32_COM_H__
#define __ESP32_COM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *                          PROTOCOL CONSTANTS
 * ============================================================================ */

#define ESP32_COM_START_BYTE_1      0xAA
#define ESP32_COM_START_BYTE_2      0xBB
#define ESP32_COM_MAX_PAYLOAD       128
#define ESP32_COM_HEADER_LEN        4
#define ESP32_COM_CRC_LEN           2
#define ESP32_COM_MIN_PACKET_LEN    (ESP32_COM_HEADER_LEN + ESP32_COM_CRC_LEN)

/* ============================================================================
 *                          PACKET TYPES (Commands → STM32)
 * ============================================================================ */

#define PKT_TYPE_SET_TUNING         0x01    /**< Set LQR tuning parameters */
#define PKT_TYPE_GET_STATE          0x02    /**< Request robot state */
#define PKT_TYPE_SET_MODE           0x03    /**< Set robot control mode */
#define PKT_TYPE_SET_ENABLE         0x04    /**< Enable/disable motors */
#define PKT_TYPE_MOTOR_CMD          0x05    /**< Direct motor command */
#define PKT_TYPE_HEARTBEAT          0x06    /**< Heartbeat / connection check */
#define PKT_TYPE_SET_SPEED          0x07    /**< Set target speed */
#define PKT_TYPE_SET_POSITION       0x08    /**< Set target joint positions */
#define PKT_TYPE_M0601C_CMD        0x09    /**< M0601C-specific command (accTime/brake) */
#define PKT_TYPE_DRIVE_CMD          0x0A    /**< Remote drive reference command */
#define PKT_TYPE_REMOTE_BTN        0x10    /**< Remote controller button event */
#define PKT_TYPE_SYS_RESET          0x20    /**< System reset command */
#define PKT_TYPE_FW_VERSION         0x30    /**< Request firmware version */

/* ============================================================================
 *                          PACKET TYPES (Responses → ESP32)
 * ============================================================================ */

#define PKT_TYPE_ACK                0x81    /**< Acknowledge with status */
#define PKT_TYPE_STATE_DATA         0x82    /**< Robot state data */
#define PKT_TYPE_TUNING_DATA        0x83    /**< Current tuning parameters */
#define PKT_TYPE_MOTOR_FB           0x84    /**< Individual motor feedback */
#define PKT_TYPE_ERROR              0x8F    /**< Error response */
#define PKT_TYPE_DEBUG_STREAM       0xF0    /**< Debug data stream */

/* ============================================================================
 *                          ERROR CODES
 * ============================================================================ */

#define ESP32_ERR_NONE              0x00
#define ESP32_ERR_CRC               0x01
#define ESP32_ERR_UNKNOWN_CMD       0x02
#define ESP32_ERR_BUSY              0x03
#define ESP32_ERR_TIMEOUT           0x04
#define ESP32_ERR_INVALID_PARAM     0x05
#define ESP32_ERR_DISABLED          0x06

/* ============================================================================
 *                          DRIVE COMMAND STATE
 * ============================================================================ */

#define ESP32_DRIVE_TIMEOUT_MS      300U

extern volatile uint8_t  g_esp32_drive_enabled;
extern volatile int16_t  g_esp32_drive_throttle;     /* -100..100 */
extern volatile int16_t  g_esp32_drive_turn;         /* -100..100 */
extern volatile int16_t  g_esp32_drive_max_rpm;      /* 0..1000 */
extern volatile uint32_t g_esp32_drive_last_tick;

/* ============================================================================
 *                          DATA STRUCTURES
 * ============================================================================ */

/**
 * @brief ESP32 communication packet (raw)
 */
typedef struct __attribute__((packed)) {
    uint8_t  start1;            /**< Start byte 1 (0xAA) */
    uint8_t  start2;            /**< Start byte 2 (0xBB) */
    uint8_t  type;              /**< Packet type */
    uint8_t  length;            /**< Payload length */
    uint8_t  payload[ESP32_COM_MAX_PAYLOAD]; /**< Payload data */
    uint16_t crc;               /**< CRC-16 (CCITT) of type+length+payload */
} ESP32_Packet_t;

/**
 * @brief ESP32 communication interface state
 */
typedef struct {
    UART_HandleTypeDef *huart;  /**< UART handle (USART3) */
    bool                initialized;

    /* RX state machine */
    volatile enum {
        ESP32_RX_IDLE,
        ESP32_RX_START1,
        ESP32_RX_START2,
        ESP32_RX_TYPE,
        ESP32_RX_LENGTH,
        ESP32_RX_PAYLOAD,
        ESP32_RX_CRC1,
        ESP32_RX_CRC2,
        ESP32_RX_COMPLETE
    } rx_state;

    volatile uint8_t    rx_buffer[ESP32_COM_HEADER_LEN + ESP32_COM_MAX_PAYLOAD + ESP32_COM_CRC_LEN];
    volatile uint16_t   rx_index;
    volatile uint8_t    rx_payload_length;
    volatile bool       packet_ready;

    /* TX buffer */
    uint8_t             tx_buffer[ESP32_COM_HEADER_LEN + ESP32_COM_MAX_PAYLOAD + ESP32_COM_CRC_LEN];

    /* Status */
    uint32_t            packets_received;
    uint32_t            packets_sent;
    uint32_t            crc_errors;
    uint32_t            last_heartbeat;
    bool                esp32_online;
} ESP32_COM_t;

/* ============================================================================
 *                          FUNCTION PROTOTYPES
 * ============================================================================ */

/* Initialization */
void ESP32_COM_Init(UART_HandleTypeDef *huart);
void ESP32_COM_StartRx(void);

/* Packet building / sending */
HAL_StatusTypeDef ESP32_COM_SendPacket(uint8_t type, uint8_t *data, uint8_t len);
HAL_StatusTypeDef ESP32_COM_SendRemoteBtn(uint8_t buttons, uint8_t prev_buttons);
HAL_StatusTypeDef ESP32_COM_SendAck(uint8_t error_code);
HAL_StatusTypeDef ESP32_COM_SendStateData(void);
HAL_StatusTypeDef ESP32_COM_SendMotorFeedback(uint8_t motor_id);

/* RX processing */
bool ESP32_COM_IsPacketReady(void);
uint8_t ESP32_COM_GetPacketType(void);
uint8_t* ESP32_COM_GetPacketPayload(void);
uint8_t ESP32_COM_GetPacketLength(void);
void ESP32_COM_ProcessPacket(void);
void ESP32_COM_ResetRx(void);

/* UART interrupt handler */
void ESP32_COM_UART_IRQHandler(uint8_t byte);

/* Status */
bool ESP32_COM_IsOnline(void);
void ESP32_COM_Update(void);

/* CRC */
uint16_t ESP32_COM_CRC16(const volatile uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __ESP32_COM_H__ */
