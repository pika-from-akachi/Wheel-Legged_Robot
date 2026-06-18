#include "wheel_app.h"

#include <stdio.h>

#include "esp_log.h"

#include "fan_pwm.h"
#include "light_pwm.h"
#include "screen_ui.h"
#include "telemetry_bridge.h"
#include "web_server.h"
#include "wheel_bsp.h"

static const char *TAG = "WHEEL_APP";

static bool s_initialized;

static const char *json_bool(bool value)
{
    return value ? "true" : "false";
}

static const char *safe_error(const char *value)
{
    return value == NULL ? "" : value;
}

esp_err_t wheel_app_init(void)
{
    fan_pwm_config_t fan_config = wheel_bsp_fan_pwm_config();
    light_pwm_config_t light_config = wheel_bsp_light_pwm_config();
    screen_ui_config_t screen_config = wheel_bsp_screen_config();

    esp_err_t ret = fan_pwm_init(&fan_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "fan PWM init: %s", esp_err_to_name(ret));
    }
    ret = light_pwm_init(&light_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "light PWM init: %s", esp_err_to_name(ret));
    }
    ret = screen_ui_init(&screen_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "screen UI init: %s", esp_err_to_name(ret));
    }

    s_initialized = true;
    return ESP_OK;
}

void wheel_app_tick(void)
{
    if (!s_initialized) {
        return;
    }

    fan_pwm_tick();
    light_pwm_tick();
    screen_ui_tick();

    web_server_status_t web_status;
    telemetry_bridge_status_t bridge_status;
    web_server_status(&web_status);
    telemetry_bridge_status(&bridge_status);

    screen_ui_snapshot_t snapshot = {
        .wifi_ready = web_status.wifi_ready,
        .stm32_uart_ready = web_status.stm32_uart_ready,
        .ws_clients = web_status.ws_client_count,
        .stm32_rx_packets = bridge_status.stm32_packets_forwarded,
    };
    screen_ui_render_snapshot(&snapshot);
}

esp_err_t wheel_app_status_json(char *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    web_server_status_t web;
    telemetry_bridge_status_t bridge;
    fan_pwm_status_t fan;
    light_pwm_status_t light;
    screen_ui_status_t screen;

    web_server_status(&web);
    telemetry_bridge_status(&bridge);
    fan_pwm_status(&fan);
    light_pwm_status(&light);
    screen_ui_status(&screen);

    const int written = snprintf(buf,
                                 len,
                                 "{"
                                 "\"status\":\"ok\","
                                 "\"wifi\":{\"ap_ready\":%s,\"ssid\":\"%s\",\"ip\":\"192.168.4.1\"},"
                                 "\"stm32_bridge\":{\"uart_ready\":%s,\"uart_port\":%d,"
                                 "\"ws_clients\":%lu,\"commands\":%lu,\"raw_commands\":%lu,"
                                 "\"packets_forwarded\":%lu,\"bytes_forwarded\":%lu,"
                                 "\"uart_packets_sent\":%lu,\"command_errors\":%lu,"
                                 "\"last_error\":\"%s\"},"
                                 "\"fan_pwm\":{\"enabled\":%s,\"initialized\":%s,\"gpio\":%d,"
                                 "\"duty_percent\":%u,\"last_error\":\"%s\"},"
                                 "\"light_pwm\":{\"enabled\":%s,\"initialized\":%s,\"gpio\":%d,"
                                 "\"mode\":\"%s\",\"duty_percent\":%u,\"last_error\":\"%s\"},"
                                 "\"screen_ui\":{\"enabled\":%s,\"initialized\":%s,\"playing\":%s,"
                                 "\"spi_host\":%d,\"sck_gpio\":%d,\"cs_gpio\":%d,"
                                 "\"data0_gpio\":%d,\"data1_gpio\":%d,\"data2_gpio\":%d,"
                                 "\"data3_gpio\":%d,\"reset_gpio\":%d,\"te_gpio\":%d,"
                                 "\"backlight_gpio\":%d,\"frames_rendered\":%lu,"
                                 "\"frames_dropped\":%lu,\"decode_errors\":%lu,"
                                 "\"last_frame_us\":%lu,\"active_asset\":\"%s\","
                                 "\"last_error\":\"%s\"}"
                                 "}",
                                 json_bool(web.wifi_ready),
                                 web.wifi_ssid,
                                 json_bool(web.stm32_uart_ready),
                                 (int)bridge.uart_port,
                                 (unsigned long)web.ws_client_count,
                                 (unsigned long)bridge.ws_text_commands,
                                 (unsigned long)bridge.ws_raw_commands,
                                 (unsigned long)bridge.stm32_packets_forwarded,
                                 (unsigned long)bridge.stm32_bytes_forwarded,
                                 (unsigned long)bridge.uart_packets_sent,
                                 (unsigned long)bridge.command_errors,
                                 safe_error(bridge.last_error),
                                 json_bool(fan.enabled),
                                 json_bool(fan.initialized),
                                 fan.gpio,
                                 fan.duty_percent,
                                 safe_error(fan.last_error),
                                 json_bool(light.enabled),
                                 json_bool(light.initialized),
                                 light.gpio,
                                 light_pwm_mode_name(light.mode),
                                 light.duty_percent,
                                 safe_error(light.last_error),
                                 json_bool(screen.enabled),
                                 json_bool(screen.initialized),
                                 json_bool(screen.playing),
                                 screen.spi_host,
                                 screen.sck_gpio,
                                 screen.cs_gpio,
                                 screen.data0_gpio,
                                 screen.data1_gpio,
                                 screen.data2_gpio,
                                 screen.data3_gpio,
                                 screen.reset_gpio,
                                 screen.te_gpio,
                                 screen.backlight_gpio,
                                 (unsigned long)screen.frames_rendered,
                                 (unsigned long)screen.frames_dropped,
                                 (unsigned long)screen.decode_errors,
                                 (unsigned long)screen.last_frame_us,
                                 safe_error(screen.active_asset),
                                 safe_error(screen.last_error));

    if (written < 0 || (size_t)written >= len) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
