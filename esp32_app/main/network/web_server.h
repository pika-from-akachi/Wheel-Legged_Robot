#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool wifi_ready;
    bool spiffs_ready;
    bool http_ready;
    bool stm32_uart_ready;
    uint32_t ws_client_count;
    uint32_t ws_total_connections;
    const char *wifi_ssid;
} web_server_status_t;

esp_err_t web_server_start(void);
esp_err_t web_server_status(web_server_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */
