#include "web_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "telemetry_bridge.h"
#include "wheel_app.h"
#include "wheel_bsp.h"

#define WIFI_AP_SSID          "WheelRobot-Tuning"
#define WIFI_AP_PASS          "12345678"
#define WIFI_AP_MAX_CONN      4
#define WS_MAX_CLIENTS        4

static const char *TAG = "WEB_SERVER";

typedef struct {
    int fd;
    bool active;
} ws_client_t;

static ws_client_t s_ws_clients[WS_MAX_CLIENTS];
static uint32_t s_ws_total_connections;
static httpd_handle_t s_httpd;
static web_server_status_t s_status = {
    .wifi_ssid = WIFI_AP_SSID,
};

static void remove_ws_client(int fd)
{
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (s_ws_clients[i].active && s_ws_clients[i].fd == fd) {
            s_ws_clients[i].active = false;
            s_ws_clients[i].fd = -1;
            if (s_status.ws_client_count > 0) {
                s_status.ws_client_count--;
            }
            return;
        }
    }
}

static void add_ws_client(int fd)
{
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (s_ws_clients[i].active && s_ws_clients[i].fd == fd) {
            return;
        }
    }
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (!s_ws_clients[i].active) {
            s_ws_clients[i].active = true;
            s_ws_clients[i].fd = fd;
            s_status.ws_client_count++;
            s_ws_total_connections++;
            s_status.ws_total_connections = s_ws_total_connections;
            return;
        }
    }
    ESP_LOGW(TAG, "No free WS client slots for fd=%d", fd);
}

static esp_err_t ws_broadcast(const uint8_t *data, uint16_t len, bool binary, void *ctx)
{
    (void)ctx;
    if (s_httpd == NULL || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t last_error = ESP_OK;
    httpd_ws_frame_t ws_pkt = {
        .final = true,
        .fragmented = false,
        .type = binary ? HTTPD_WS_TYPE_BINARY : HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)data,
        .len = len,
    };

    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (!s_ws_clients[i].active) {
            continue;
        }
        esp_err_t ret = httpd_ws_send_data(s_httpd, s_ws_clients[i].fd, &ws_pkt);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "WS client fd=%d disconnected: %s",
                     s_ws_clients[i].fd,
                     esp_err_to_name(ret));
            remove_ws_client(s_ws_clients[i].fd);
            last_error = ret;
        }
    }
    return last_error;
}

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
    s_status.wifi_ready = true;

    ESP_LOGI(TAG, "WiFi AP started: SSID='%s', password='%s'",
             WIFI_AP_SSID, WIFI_AP_PASS);
    ESP_LOGI(TAG, "AP IP: 192.168.4.1");
}

static void uart_stm_init(void)
{
    wheel_bsp_uart_config_t config = wheel_bsp_stm32_uart_config();
    uart_config_t uart_config = {
        .baud_rate = config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(config.port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(config.port, config.tx_gpio, config.rx_gpio, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(config.port, config.rx_buffer_size * 2, 0, 0, NULL, 0));

    telemetry_bridge_config_t bridge_config = {
        .uart_port = config.port,
        .ws_send = ws_broadcast,
        .ws_send_ctx = NULL,
    };
    ESP_ERROR_CHECK(telemetry_bridge_init(&bridge_config));
    s_status.stm32_uart_ready = true;
}

static void spiffs_init(void)
{
    if (esp_spiffs_mounted(NULL)) {
        s_status.spiffs_ready = true;
        ESP_LOGI(TAG, "SPIFFS already mounted at /www");
        return;
    }

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/www",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret == ESP_ERR_INVALID_STATE && esp_spiffs_mounted(NULL)) {
        s_status.spiffs_ready = true;
        ESP_LOGI(TAG, "SPIFFS already mounted at /www");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
    } else {
        s_status.spiffs_ready = true;
        ESP_LOGI(TAG, "SPIFFS mounted at /www");
    }
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    esp_err_t ret = ESP_FAIL;
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

static esp_err_t state_get_handler(httpd_req_t *req)
{
    char json[1536];
    esp_err_t ret = wheel_app_status_json(json, sizeof(json));
    if (ret != ESP_OK) {
        httpd_resp_send_500(req);
        return ret;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

static esp_err_t ws_post_handshake_handler(httpd_req_t *req)
{
    const int fd = httpd_req_to_sockfd(req);
    add_ws_client(fd);
    ESP_LOGI(TAG, "WebSocket connected fd=%d", fd);
    return ESP_OK;
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    const int fd = httpd_req_to_sockfd(req);

    httpd_ws_frame_t frame = {0};
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        remove_ws_client(fd);
        return ret;
    }

    if (frame.len == 0) {
        return ESP_OK;
    }
    if (frame.len > 512) {
        ESP_LOGW(TAG, "WS frame too large: %d", (int)frame.len);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t *buf = calloc(frame.len + 1, 1);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    frame.payload = buf;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret == ESP_OK) {
        if (frame.type == HTTPD_WS_TYPE_TEXT) {
            telemetry_bridge_handle_ws_text(frame.payload, frame.len);
        } else if (frame.type == HTTPD_WS_TYPE_BINARY) {
            wheel_bsp_uart_config_t config = wheel_bsp_stm32_uart_config();
            uart_write_bytes(config.port, (const char *)frame.payload, frame.len);
        } else if (frame.type == HTTPD_WS_TYPE_CLOSE) {
            remove_ws_client(fd);
        }

        httpd_ws_frame_t echo = {
            .final = true,
            .fragmented = false,
            .type = frame.type,
            .payload = frame.payload,
            .len = frame.len,
        };
        httpd_ws_send_frame(req, &echo);
    } else {
        remove_ws_client(fd);
    }

    free(buf);
    return ret;
}

static void start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 8;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    if (httpd_start(&s_httpd, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_httpd, &uri_root);

    httpd_uri_t uri_state = {
        .uri = "/state",
        .method = HTTP_GET,
        .handler = state_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_httpd, &uri_state);

    httpd_uri_t uri_ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
        .handle_ws_control_frames = true,
        .ws_post_handshake_cb = ws_post_handshake_handler,
    };
    httpd_register_uri_handler(s_httpd, &uri_ws);

    s_status.http_ready = true;
    ESP_LOGI(TAG, "HTTP/WebSocket server started");
}

static void uart_to_ws_task(void *arg)
{
    (void)arg;
    wheel_bsp_uart_config_t config = wheel_bsp_stm32_uart_config();
    uint8_t data[256];

    while (1) {
        const int len = uart_read_bytes(config.port, data, sizeof(data), pdMS_TO_TICKS(1));
        if (len > 0) {
            telemetry_bridge_handle_stm32_bytes(data, (uint16_t)len);
        }
        wheel_app_tick();
        taskYIELD();
    }
}

esp_err_t web_server_start(void)
{
    spiffs_init();
    wifi_init_ap();
    uart_stm_init();
    start_http_server();
    xTaskCreate(uart_to_ws_task, "uart_ws", 4096, NULL, 5, NULL);
    return ESP_OK;
}

esp_err_t web_server_status(web_server_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_status;
    return ESP_OK;
}
