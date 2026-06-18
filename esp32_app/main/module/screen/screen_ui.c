#include "screen_ui.h"

#include "esp_log.h"

#include "screen_player.h"

static const char *TAG = "SCREEN_UI";
static screen_ui_status_t s_status;
static screen_ui_config_t s_config;

static void update_player_status(void)
{
    screen_player_status_t player;
    if (screen_player_status(&player) != ESP_OK) {
        return;
    }

    s_status.initialized = player.initialized;
    s_status.playing = player.playing;
    s_status.frames_rendered = player.frames_rendered;
    s_status.frames_dropped = player.frames_dropped;
    s_status.decode_errors = player.decode_errors;
    s_status.last_frame_us = player.last_frame_us;
    s_status.active_asset = player.active_asset;
    s_status.last_error = player.last_error;
}

esp_err_t screen_ui_init(const screen_ui_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;
    s_status = (screen_ui_status_t) {
        .enabled = config->enabled,
        .initialized = false,
        .playing = false,
        .spi_host = config->spi_host,
        .sck_gpio = config->sck_gpio,
        .cs_gpio = config->cs_gpio,
        .data0_gpio = config->data0_gpio,
        .data1_gpio = config->data1_gpio,
        .data2_gpio = config->data2_gpio,
        .data3_gpio = config->data3_gpio,
        .reset_gpio = config->reset_gpio,
        .te_gpio = config->te_gpio,
        .backlight_gpio = config->backlight_gpio,
        .last_error = config->enabled ? "not_initialized" : "disabled",
    };

    if (!config->enabled) {
        return ESP_OK;
    }

    esp_err_t ret = screen_player_init(config);
    update_player_status();
    if (ret != ESP_OK) {
        s_status.last_error = "screen_player_init_failed";
        return ret;
    }

    ret = screen_player_start_eyes();
    update_player_status();
    if (ret != ESP_OK) {
        s_status.last_error = "screen_eyes_start_failed";
        return ret;
    }

    ESP_LOGI(TAG, "Screen RoboEyes animation active");
    return ESP_OK;
}

void screen_ui_tick(void)
{
    if (!s_config.enabled) {
        return;
    }
    update_player_status();
}

esp_err_t screen_ui_status(screen_ui_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_status;
    return ESP_OK;
}

esp_err_t screen_ui_render_snapshot(const screen_ui_snapshot_t *snapshot)
{
    (void)snapshot;
    if (!s_status.enabled) {
        s_status.last_error = "disabled";
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_status.initialized) {
        s_status.last_error = "uninitialized";
        return ESP_ERR_INVALID_STATE;
    }
    update_player_status();
    return ESP_OK;
}
