/**
 * @file    main.c
 * @brief   ESP32-S3 entrypoint for Wheel-Legged Robot auxiliary runtime.
 *
 * The ESP32 owns wireless tuning, WebSocket telemetry forwarding, and local
 * auxiliary modules. The STM32 remains the control-loop owner.
 */

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "web_server.h"
#include "wheel_bsp.h"
#include "wheel_app.h"

static const char *TAG = "WHEEL_MAIN";

static void force_screen_backlight_on(void)
{
    wheel_bsp_screen_config_t screen = wheel_bsp_screen_config();
    if (screen.backlight_gpio < 0) {
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << screen.backlight_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "screen backlight gpio config failed: %s", esp_err_to_name(ret));
        return;
    }

    gpio_set_level(screen.backlight_gpio, screen.backlight_active_high ? 1 : 0);
    ESP_LOGI(TAG, "screen backlight forced on: gpio=%d active_%s",
             screen.backlight_gpio,
             screen.backlight_active_high ? "high" : "low");
}

void app_main(void)
{
    force_screen_backlight_on();

    ESP_LOGI(TAG, "Wheel-Legged Robot ESP32-S3 auxiliary runtime v1.0");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(wheel_app_init());
    ESP_ERROR_CHECK(web_server_start());

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  Wheel-Legged Robot ESP32-S3 Runtime");
    ESP_LOGI(TAG, "  WiFi: WheelRobot-Tuning");
    ESP_LOGI(TAG, "  URL: http://192.168.4.1");
    ESP_LOGI(TAG, "===========================================");
}
