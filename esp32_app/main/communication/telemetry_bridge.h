#ifndef TELEMETRY_BRIDGE_H
#define TELEMETRY_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*telemetry_bridge_ws_send_fn_t)(const uint8_t *data,
                                                   uint16_t len,
                                                   bool binary,
                                                   void *ctx);

typedef struct {
    uart_port_t uart_port;
    telemetry_bridge_ws_send_fn_t ws_send;
    void *ws_send_ctx;
} telemetry_bridge_config_t;

typedef struct {
    bool initialized;
    uart_port_t uart_port;
    uint32_t ws_text_commands;
    uint32_t ws_raw_commands;
    uint32_t stm32_packets_forwarded;
    uint32_t stm32_bytes_forwarded;
    uint32_t uart_packets_sent;
    uint32_t json_errors;
    uint32_t command_errors;
    const char *last_error;
} telemetry_bridge_status_t;

esp_err_t telemetry_bridge_init(const telemetry_bridge_config_t *config);
esp_err_t telemetry_bridge_handle_ws_text(const uint8_t *data, size_t len);
esp_err_t telemetry_bridge_handle_stm32_bytes(const uint8_t *data, uint16_t len);
esp_err_t telemetry_bridge_status(telemetry_bridge_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_BRIDGE_H */
