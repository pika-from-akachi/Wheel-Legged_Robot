#ifndef SCREEN_UI_H
#define SCREEN_UI_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "wheel_bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef wheel_bsp_screen_config_t screen_ui_config_t;

typedef struct {
    bool enabled;
    bool initialized;
    bool playing;
    int spi_host;
    int sck_gpio;
    int cs_gpio;
    int data0_gpio;
    int data1_gpio;
    int data2_gpio;
    int data3_gpio;
    int reset_gpio;
    int te_gpio;
    int backlight_gpio;
    uint32_t frames_rendered;
    uint32_t frames_dropped;
    uint32_t decode_errors;
    uint32_t last_frame_us;
    const char *active_asset;
    const char *last_error;
} screen_ui_status_t;

typedef struct {
    bool wifi_ready;
    bool stm32_uart_ready;
    uint32_t ws_clients;
    uint32_t stm32_rx_packets;
} screen_ui_snapshot_t;

esp_err_t screen_ui_init(const screen_ui_config_t *config);
void screen_ui_tick(void);
esp_err_t screen_ui_status(screen_ui_status_t *out);
esp_err_t screen_ui_render_snapshot(const screen_ui_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* SCREEN_UI_H */
