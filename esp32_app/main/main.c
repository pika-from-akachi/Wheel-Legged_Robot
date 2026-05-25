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
#include <stdlib.h>
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

/* JSON parsing */
#include "cJSON.h"

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
 *                          STM32 PROTOCOL CONSTANTS
 * ============================================================================ */

#define STM32_PKT_START1        0xAA
#define STM32_PKT_START2        0xBB
#define STM32_PKT_HEADER_LEN    4
#define STM32_PKT_CRC_LEN       2
#define STM32_MAX_PAYLOAD       128

/* Packet types (commands to STM32) */
#define PKT_TYPE_SET_TUNING     0x01
#define PKT_TYPE_SET_MODE       0x03
#define PKT_TYPE_SET_ENABLE     0x04
#define PKT_TYPE_M0601C_CMD     0x09

/* ============================================================================
 *                          CRC-16 CCITT
 * ============================================================================ */

static const uint16_t s_crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

static uint16_t stm32_crc16(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ s_crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

/* ============================================================================
 *                          STM32 PACKET BUILDER
 * ============================================================================ */

static void build_stm32_packet(uint8_t *buf, uint16_t *len, uint8_t type, uint8_t *payload, uint8_t payload_len)
{
    uint16_t idx = 0;
    buf[idx++] = STM32_PKT_START1;
    buf[idx++] = STM32_PKT_START2;
    buf[idx++] = type;
    buf[idx++] = payload_len;
    if (payload_len > 0 && payload != NULL) {
        memcpy(&buf[idx], payload, payload_len);
        idx += payload_len;
    }
    uint16_t crc = stm32_crc16(&buf[2], payload_len + 2);
    buf[idx++] = (uint8_t)(crc >> 8);
    buf[idx++] = (uint8_t)(crc & 0xFF);
    *len = idx;
}

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

/* WebSocket handler — parses JSON commands, forwards as STM32 binary packets */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket connection");
        return ESP_OK;
    }

    /* Handle WebSocket frames */
    uint8_t buf[256];
    int fd = httpd_req_to_sockfd(req);
    int len = recv(fd, buf, sizeof(buf) - 1, 0);

    if (len > 0) {
        buf[len] = 0;

        /* Try to parse as JSON command from browser */
        cJSON *root = cJSON_Parse((char *)buf);
        if (root != NULL) {
            uint8_t stm32_pkt[STM32_PKT_HEADER_LEN + STM32_MAX_PAYLOAD + STM32_PKT_CRC_LEN];
            uint16_t pkt_len = 0;
            bool built = false;

            cJSON *type = cJSON_GetObjectItem(root, "type");
            if (type != NULL && cJSON_IsString(type)) {
                /* Handle tuning command */
                if (strcmp(type->valuestring, "tuning") == 0) {
                    cJSON *params = cJSON_GetObjectItem(root, "params");
                    if (params != NULL) {
                        /* Pack LQR_TuningParams_t as floats */
                        uint8_t payload[40];
                        uint8_t plen = 0;
                        float val;

                        val = cJSON_GetObjectItem(params, "Q_body_angle")->valuedouble;
                        memcpy(&payload[plen], &val, 4); plen += 4;
                        val = cJSON_GetObjectItem(params, "Q_body_rate")->valuedouble;
                        memcpy(&payload[plen], &val, 4); plen += 4;
                        val = cJSON_GetObjectItem(params, "Q_wheel_position")->valuedouble;
                        memcpy(&payload[plen], &val, 4); plen += 4;
                        val = cJSON_GetObjectItem(params, "Q_wheel_velocity")->valuedouble;
                        memcpy(&payload[plen], &val, 4); plen += 4;
                        val = cJSON_GetObjectItem(params, "R_joint_torque")->valuedouble;
                        memcpy(&payload[plen], &val, 4); plen += 4;
                        val = cJSON_GetObjectItem(params, "R_wheel_torque")->valuedouble;
                        memcpy(&payload[plen], &val, 4); plen += 4;

                        build_stm32_packet(stm32_pkt, &pkt_len, PKT_TYPE_SET_TUNING, payload, plen);
                        built = true;
                    }
                }
                /* Handle enable/disable command */
                else if (strcmp(type->valuestring, "command") == 0) {
                    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
                    if (cmd != NULL && cJSON_IsString(cmd)) {
                        if (strcmp(cmd->valuestring, "enable") == 0) {
                            uint8_t payload[] = {1};
                            build_stm32_packet(stm32_pkt, &pkt_len, PKT_TYPE_SET_ENABLE, payload, 1);
                            built = true;
                        } else if (strcmp(cmd->valuestring, "disable") == 0) {
                            uint8_t payload[] = {0};
                            build_stm32_packet(stm32_pkt, &pkt_len, PKT_TYPE_SET_ENABLE, payload, 1);
                            built = true;
                        } else if (strcmp(cmd->valuestring, "set_mode") == 0) {
                            cJSON *mode = cJSON_GetObjectItem(root, "mode");
                            if (mode != NULL && cJSON_IsString(mode)) {
                                uint8_t mode_id = 0;
                                if (strcmp(mode->valuestring, "driving") == 0) mode_id = 1;
                                else if (strcmp(mode->valuestring, "sitting") == 0) mode_id = 2;
                                else if (strcmp(mode->valuestring, "calibration") == 0) mode_id = 5;
                                /* default: standing = 0 */
                                build_stm32_packet(stm32_pkt, &pkt_len, PKT_TYPE_SET_MODE, &mode_id, 1);
                                built = true;
                            }
                        } else if (strcmp(cmd->valuestring, "m0601c_speed") == 0) {
                            /* Direct speed command for M0601C testing */
                            cJSON *speed_j = cJSON_GetObjectItem(root, "speed");
                            cJSON *motor_j = cJSON_GetObjectItem(root, "motor");
                            if (speed_j != NULL && cJSON_IsNumber(speed_j)) {
                                int16_t speed_val = (int16_t)speed_j->valuedouble;
                                int motor_count = 2;
                                if (motor_j != NULL && cJSON_IsString(motor_j) &&
                                    strcmp(motor_j->valuestring, "left") == 0) {
                                    motor_count = 1;
                                }
                                for (int m = 0; m < motor_count; m++) {
                                    uint8_t pl[4] = { (uint8_t)m, 0, 0, 0 };
                                    pl[1] = (uint8_t)(speed_val >> 8);
                                    pl[2] = (uint8_t)(speed_val & 0xFF);
                                    build_stm32_packet(stm32_pkt, &pkt_len, PKT_TYPE_M0601C_CMD, pl, 4);
                                    uart_write_bytes(UART_STM_PORT, (const char *)stm32_pkt, pkt_len);
                                }
                                built = false; /* Already sent above */
                            }
                        }
                    }
                }
                /* Handle M0601C tuning command */
                else if (strcmp(type->valuestring, "m0601c_cmd") == 0) {
                    cJSON *motor_idx = cJSON_GetObjectItem(root, "motor_idx");
                    cJSON *cmd_type = cJSON_GetObjectItem(root, "cmd_type");
                    cJSON *value = cJSON_GetObjectItem(root, "value");
                    if (motor_idx != NULL && cmd_type != NULL && value != NULL) {
                        uint8_t payload[4];
                        payload[0] = (uint8_t)motor_idx->valuedouble;
                        payload[1] = (uint8_t)cmd_type->valuedouble;
                        int16_t v = (int16_t)value->valuedouble;
                        payload[2] = (uint8_t)(v >> 8);
                        payload[3] = (uint8_t)(v & 0xFF);
                        build_stm32_packet(stm32_pkt, &pkt_len, PKT_TYPE_M0601C_CMD, payload, 4);
                        built = true;
                    }
                }
            }

            cJSON_Delete(root);

            /* Send built packet to STM32 */
            if (built) {
                uart_write_bytes(UART_STM_PORT, (const char *)stm32_pkt, pkt_len);
            }
        } else {
            /* Not JSON — forward raw bytes to STM32 (legacy support) */
            uart_write_bytes(UART_STM_PORT, (const char *)buf, len);
        }

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
