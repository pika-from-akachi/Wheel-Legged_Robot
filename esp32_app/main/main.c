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

#include "web_server.h"
#include "wheel_app.h"

static const char *TAG = "WHEEL_MAIN";

void app_main(void)
{
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
