/**
  ******************************************************************************
  * @file    can_rx_handler.c
  * @brief   CAN Receive Handler for EL05 Motor
  * @note    This file provides complete CAN RX data parsing implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "el05_motor.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
extern EL05_MotorHandle_t motor1;  // Motor handle (defined in main.c)

/* Private function prototypes -----------------------------------------------*/
static void EL05_ParseFeedback(uint8_t motor_id, uint8_t *data);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Parse EL05 motor feedback data (Type 2)
 * @param motor_id: Motor CAN ID
 * @param data: Pointer to data buffer (8 bytes)
 * @retval None
 *
 * @note Data format (Type 2 - Motor Feedback):
 *   Byte 0: Motor ID
 *   Byte 1: Fault status
 *   Byte 2-3: Position (16-bit)
 *   Byte 4-5: Velocity (12-bit) + Temperature (4-bit)
 *   Byte 6-7: Torque (12-bit) + reserved (4-bit)
 */
static void EL05_ParseFeedback(uint8_t motor_id, uint8_t *data)
{
    // Check if this is our motor
    if (motor_id == motor1.can_id)
    {
        // Update timestamp
        motor1.last_update_time = HAL_GetTick();
        motor1.is_online = 1;

        // Parse motor ID
        motor1.feedback.id = data[0];

        // Parse fault status
        motor1.feedback.fault = data[1];

        // Parse position (16-bit, range: -12.5 ~ 12.5 rad)
        uint16_t pos_uint = ((uint16_t)data[2] << 8) | data[3];
        motor1.feedback.position = uint_to_float(pos_uint, EL05_P_MIN, EL05_P_MAX, 16);

        // Parse velocity (12-bit, range: -30 ~ 30 rad/s)
        uint16_t vel_uint = ((uint16_t)data[4] << 4) | (data[5] >> 4);
        motor1.feedback.velocity = uint_to_float(vel_uint, EL05_V_MIN, EL05_V_MAX, 12);

        // Parse temperature (4-bit, range: 0 ~ 255℃)
        // Note: Temperature is in the lower 4 bits of byte 5
        // This is a simplified interpretation, actual format may vary
        uint8_t temp_raw = data[5] & 0x0F;
        motor1.feedback.temperature = (float)temp_raw * 16.0f;  // Scale factor

        // Parse torque (12-bit, range: -18 ~ 18 N·m)
        uint16_t torque_uint = ((uint16_t)data[6] << 4) | (data[7] >> 4);
        motor1.feedback.torque = uint_to_float(torque_uint, EL05_T_MIN, EL05_T_MAX, 12);

        // Update motor state based on fault status
        if (motor1.feedback.fault != 0) {
            motor1.state = EL05_STATE_ERROR;
        }
    }
}

/**
 * @brief Enhanced CAN RX callback with complete data parsing
 * @param hcan: Pointer to CAN handle
 * @retval None
 */
void EL05_CAN_RxHandler(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_StatusTypeDef status;

    // Process FIFO 0
    status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (status == HAL_OK)
    {
        // Check if extended frame
        if (rx_header.IDE == CAN_ID_EXT)
        {
            uint32_t ext_id = rx_header.ExtId;

            // Parse CAN ID: [Type][ID]
            uint8_t msg_type = (ext_id >> 8) & 0xFF;
            uint8_t motor_id = ext_id & 0xFF;

            // Process based on message type
            switch (msg_type)
            {
                case EL05_TYPE_FEEDBACK:
                    // Type 2: Motor feedback data
                    EL05_ParseFeedback(motor_id, rx_data);
                    break;

                case EL05_TYPE_GET_ID:
                    // Type 0: Device ID response
                    // Handle device ID response
                    break;

                case EL05_TYPE_READ_PARAM:
                    // Type 17: Parameter read response
                    // Handle parameter read response
                    // Data format: [addr_h][addr_l][data4][data3][data2][data1][reserved][reserved]
                    if (motor_id == motor1.can_id)
                    {
                        uint16_t param_addr = ((uint16_t)rx_data[0] << 8) | rx_data[1];
                        uint32_t param_value_uint = ((uint32_t)rx_data[2] << 24) |
                                                    ((uint32_t)rx_data[3] << 16) |
                                                    ((uint32_t)rx_data[4] << 8) |
                                                    rx_data[5];
                        float param_value = *(float*)&param_value_uint;

                        // User can add custom handling here
                        // For example, store in a parameter buffer
                    }
                    break;

                case EL05_TYPE_FAULT_FEEDBACK:
                    // Type 21: Fault feedback
                    // Handle fault feedback
                    if (motor_id == motor1.can_id)
                    {
                        motor1.feedback.fault = rx_data[0];
                        motor1.state = EL05_STATE_ERROR;
                    }
                    break;

                default:
                    // Unknown message type
                    break;
            }
        }
    }

    // Process FIFO 1 (same as FIFO 0)
    status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
    if (status == HAL_OK)
    {
        if (rx_header.IDE == CAN_ID_EXT)
        {
            uint32_t ext_id = rx_header.ExtId;
            uint8_t msg_type = (ext_id >> 8) & 0xFF;
            uint8_t motor_id = ext_id & 0xFF;

            switch (msg_type)
            {
                case EL05_TYPE_FEEDBACK:
                    EL05_ParseFeedback(motor_id, rx_data);
                    break;

                // Add other cases as needed...
                default:
                    break;
            }
        }
    }
}

/**
 * @brief Override HAL CAN RX callback
 * @param hcan: Pointer to CAN handle
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    EL05_CAN_RxHandler(hcan);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    EL05_CAN_RxHandler(hcan);
}
