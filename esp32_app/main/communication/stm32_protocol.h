#ifndef STM32_PROTOCOL_H
#define STM32_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32_PKT_START1        0xAA
#define STM32_PKT_START2        0xBB
#define STM32_PKT_HEADER_LEN    4
#define STM32_PKT_CRC_LEN       2
#define STM32_MAX_PAYLOAD       128
#define STM32_MAX_PACKET_LEN    (STM32_PKT_HEADER_LEN + STM32_MAX_PAYLOAD + STM32_PKT_CRC_LEN)

#define PKT_TYPE_SET_TUNING     0x01
#define PKT_TYPE_GET_STATE      0x02
#define PKT_TYPE_SET_MODE       0x03
#define PKT_TYPE_SET_ENABLE     0x04
#define PKT_TYPE_MOTOR_CMD      0x05
#define PKT_TYPE_HEARTBEAT      0x06
#define PKT_TYPE_SET_SPEED      0x07
#define PKT_TYPE_SET_POSITION   0x08
#define PKT_TYPE_M0601C_CMD     0x09
#define PKT_TYPE_REMOTE_BTN     0x10
#define PKT_TYPE_SYS_RESET      0x20
#define PKT_TYPE_FW_VERSION     0x30

#define PKT_TYPE_ACK            0x81
#define PKT_TYPE_STATE_DATA     0x82
#define PKT_TYPE_TUNING_DATA    0x83
#define PKT_TYPE_MOTOR_FB       0x84
#define PKT_TYPE_ERROR          0x8F
#define PKT_TYPE_DEBUG_STREAM   0xF0

#define STM32_LQR_TUNING_FLOAT_COUNT 10
#define STM32_LQR_TUNING_PAYLOAD_LEN (STM32_LQR_TUNING_FLOAT_COUNT * sizeof(float))

uint16_t stm32_protocol_crc16(const uint8_t *data, uint16_t len);
esp_err_t stm32_protocol_build_packet(uint8_t type,
                                      const uint8_t *payload,
                                      uint8_t payload_len,
                                      uint8_t *out,
                                      size_t out_capacity,
                                      uint16_t *out_len);
bool stm32_protocol_packet_bounds_ok(size_t buffer_len, size_t offset, uint8_t payload_len);

#ifdef __cplusplus
}
#endif

#endif /* STM32_PROTOCOL_H */
