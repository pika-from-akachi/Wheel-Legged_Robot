/**
 * @file    main.c
 * @brief   ESP32-S3 main firmware for Wheel-Legged Robot Remote Tuning
 * @note    WiFi AP + WebSocket + UART bridge to STM32F407
 *
 * Hardware:
 *   - ESP32-S3
 *   - UART TX (GPIO43) → STM32 USART3 RX (PB11)
 *   - UART RX (GPIO44) → STM32 USART3 TX (PB10)
 *   - WiFi AP mode: SSID="WheelRobot-Tuning", password="12345678"
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/* HTTP and WebSocket server */
#include "esp_http_server.h"

static const char *TAG = "WHEEL_ROBOT";

/* ============================================================================
 *                          CONFIGURATION
 * ============================================================================ */

#define WIFI_AP_SSID          "WheelRobot-Tuning"
#define WIFI_AP_PASS          "12345678"
#define WIFI_AP_MAX_CONN      4

#define UART_STM_PORT         UART_NUM_1
#define UART_STM_TX_GPIO      43
#define UART_STM_RX_GPIO      44
#define UART_STM_BAUD         921600
#define UART_STM_BUF_SIZE     256

#define WS_MAX_CLIENTS        4
#define STATE_BROADCAST_MS    50    /* 20Hz state broadcast */

/* ============================================================================
 *                          GLOBAL STATE
 * ============================================================================ */

typedef struct {
    struct sockaddr_in addr;
    int fd;
    bool active;
} ws_client_t;

static ws_client_t ws_clients[WS_MAX_CLIENTS];
static int ws_client_count = 0;

/* Latest robot state (binary, mirrors STM32 packet format) */
static uint8_t g_robot_state_buf[128];
static uint16_t g_robot_state_len = 0;

/* Queue for UART → WebSocket forwarding */
static QueueHandle_t uart_to_ws_queue = NULL;

/* ============================================================================
 *                          WIFI INITIALIZATION
 * ============================================================================ */

static void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: SSID='%s', password='%s'",
             WIFI_AP_SSID, WIFI_AP_PASS);
    ESP_LOGI(TAG, "AP IP: 192.168.4.1");
}

/* ============================================================================
 *                          UART INITIALIZATION (STM32 COMM)
 * ============================================================================ */

static void uart_stm_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_STM_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_STM_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_STM_PORT, UART_STM_TX_GPIO, UART_STM_RX_GPIO, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(UART_STM_PORT, UART_STM_BUF_SIZE * 2, 0, 0, NULL, 0));
}

/* ============================================================================
 *                          SPIFFS INITIALIZATION
 * ============================================================================ */

static void spiffs_init(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/www",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS mounted at /www");
    }
}

/* ============================================================================
 *                          WEBSOCKET HANDLING
 * ============================================================================ */

static void ws_broadcast(const uint8_t *data, uint16_t len)
{
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (ws_clients[i].active) {
            int ret = send(ws_clients[i].fd, data, len, 0);
            if (ret < 0) {
                ESP_LOGW(TAG, "WS client %d disconnected", i);
                close(ws_clients[i].fd);
                ws_clients[i].active = false;
                ws_client_count--;
            }
        }
    }
}

/* WebSocket handler */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket connection");
        return ESP_OK;
    }

    /* Handle WebSocket frames */
    uint8_t buf[128];
    int fd = httpd_req_to_sockfd(req);
    int len = recv(fd, buf, sizeof(buf), 0);

    if (len > 0) {
        /* Forward received data to STM32 via UART */
        uart_write_bytes(UART_STM_PORT, (const char *)buf, len);

        /* Echo back for confirmation */
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = buf;
        ws_pkt.len = len;
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;
        httpd_ws_send_frame(req, &ws_pkt);
    }

    return ESP_OK;
}

/* HTTP GET handler for root */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    esp_err_t ret = ESP_FAIL;

    /* Try to serve index.html from SPIFFS */
    FILE *f = fopen("/www/index.html", "r");
    if (f != NULL) {
        fseek(f, 0, SEEK_END);
        long fsize = ftell(f);
        fseek(f, 0, SEEK_SET);

        char *content = malloc(fsize + 1);
        if (content) {
            fread(content, 1, fsize, f);
            content[fsize] = 0;
            httpd_resp_set_type(req, "text/html");
            httpd_resp_send(req, content, fsize);
            free(content);
            ret = ESP_OK;
        }
        fclose(f);
    }

    if (ret != ESP_OK) {
        httpd_resp_sendstr(req, "Wheel-Legged Robot Tuning Interface (SPIFFS not mounted)");
    }
    return ESP_OK;
}

/* State data endpoint (JSON for non-WebSocket clients) */
static esp_err_t state_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"Use WebSocket for real-time data\"}");
    return ESP_OK;
}

/* ============================================================================
 *                          HTTP SERVER
 * ============================================================================ */

static void start_http_server(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 8;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    /* Register URI handlers */
    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_root);

    httpd_uri_t uri_state = {
        .uri = "/state",
        .method = HTTP_GET,
        .handler = state_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_state);

    httpd_uri_t uri_ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true
    };
    httpd_register_uri_handler(server, &uri_ws);

    ESP_LOGI(TAG, "HTTP/WebSocket server started");
}

/* ============================================================================
 *                          UART → WS FORWARDING TASK
 * ============================================================================ */

static void uart_to_ws_task(void *arg)
{
    uint8_t data[256];
    int len;

    while (1) {
        len = uart_read_bytes(UART_STM_PORT, data, sizeof(data), pdMS_TO_TICKS(10));
        if (len > 0) {
            /* Broadcast to all WebSocket clients */
            ws_broadcast(data, len);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ============================================================================
 *                          CONFIG API (REST)
 * ============================================================================ */

/* Handle parameter updates from web frontend */
static void handle_ws_command(const uint8_t *data, uint16_t len)
{
    /* Expect JSON or binary command packet */
    /* Forward directly to STM32 via UART protocol */
    uart_write_bytes(UART_STM_PORT, (const char *)data, len);
}

/* ============================================================================
 *                          MAIN
 * ============================================================================ */

void app_main(void)
{
    ESP_LOGI(TAG, "Wheel-Legged Robot ESP32-S3 Tuning Interface v1.0");

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize SPIFFS for web files */
    spiffs_init();

    /* Initialize WiFi AP */
    wifi_init_ap();

    /* Initialize UART for STM32 communication */
    uart_stm_init();

    /* Start HTTP + WebSocket server */
    start_http_server();

    /* Create UART → WebSocket forwarding task */
    xTaskCreate(uart_to_ws_task, "uart_ws", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  Wheel-Legged Robot Tuning Interface");
    ESP_LOGI(TAG, "  WiFi: %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "  URL: http://192.168.4.1");
    ESP_LOGI(TAG, "===========================================");
}
