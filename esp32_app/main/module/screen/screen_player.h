#ifndef SCREEN_PLAYER_H
#define SCREEN_PLAYER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "wheel_bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SCREEN_PLAYER_WIDTH  360
#define SCREEN_PLAYER_HEIGHT 360

typedef wheel_bsp_screen_config_t screen_player_config_t;

typedef struct {
    bool enabled;
    bool initialized;
    bool playing;
    uint32_t frames_rendered;
    uint32_t frames_dropped;
    uint32_t decode_errors;
    uint32_t last_frame_us;
    const char *active_asset;
    const char *last_error;
} screen_player_status_t;

esp_err_t screen_player_init(const screen_player_config_t *config);
esp_err_t screen_player_start_eyes(void);
esp_err_t screen_player_draw_rgb565_rect(int x, int y, int width, int height, const void *rgb565, size_t len);
esp_err_t screen_player_fill_rgb565(uint16_t rgb565);
void screen_player_report_frame(esp_err_t ret, uint32_t elapsed_us, const char *error);
esp_err_t screen_player_status(screen_player_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* SCREEN_PLAYER_H */
