#include "telemetry_bridge.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "stm32_protocol.h"
#include "../module/light/light_pwm.h"

static const char *TAG = "TELEMETRY_BRIDGE";

typedef struct {
    telemetry_bridge_config_t config;
    telemetry_bridge_status_t status;
} telemetry_bridge_state_t;

static telemetry_bridge_state_t s_bridge;

static esp_err_t send_packet(uint8_t type, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t packet[STM32_MAX_PACKET_LEN];
    uint16_t packet_len = 0;
    esp_err_t ret = stm32_protocol_build_packet(type,
                                                payload,
                                                payload_len,
                                                packet,
                                                sizeof(packet),
                                                &packet_len);
    if (ret != ESP_OK) {
        s_bridge.status.last_error = "packet_build_failed";
        return ret;
    }

    const int written = uart_write_bytes(s_bridge.config.uart_port,
                                         (const char *)packet,
                                         packet_len);
    if (written != packet_len) {
        s_bridge.status.last_error = "uart_write_failed";
        return ESP_FAIL;
    }
    s_bridge.status.uart_packets_sent++;
    s_bridge.status.last_error = NULL;
    return ESP_OK;
}

static const char *find_json_value(const char *json, const char *key)
{
    char needle[48];
    if (snprintf(needle, sizeof(needle), "\"%s\"", key) >= (int)sizeof(needle)) {
        return NULL;
    }

    const char *pos = strstr(json, needle);
    if (pos == NULL) {
        return NULL;
    }
    pos = strchr(pos + strlen(needle), ':');
    if (pos == NULL) {
        return NULL;
    }
    pos++;
    while (*pos != '\0' && isspace((unsigned char)*pos)) {
        pos++;
    }
    return pos;
}

static bool json_get_string(const char *json, const char *key, char *out, size_t out_len)
{
    const char *pos = find_json_value(json, key);
    if (pos == NULL || *pos != '"' || out == NULL || out_len == 0) {
        return false;
    }
    pos++;

    size_t i = 0;
    while (*pos != '\0' && *pos != '"' && i + 1 < out_len) {
        if (*pos == '\\' && pos[1] != '\0') {
            pos++;
        }
        out[i++] = *pos++;
    }
    if (*pos != '"') {
        return false;
    }
    out[i] = '\0';
    return true;
}

static bool json_get_float(const char *json, const char *key, float fallback, float *out)
{
    const char *pos = find_json_value(json, key);
    if (pos == NULL) {
        *out = fallback;
        return true;
    }
    char *end = NULL;
    const double parsed = strtod(pos, &end);
    if (end == pos) {
        return false;
    }
    *out = (float)parsed;
    return true;
}

static bool json_get_int(const char *json, const char *key, int *out)
{
    const char *pos = find_json_value(json, key);
    if (pos == NULL) {
        return false;
    }
    char *end = NULL;
    const long parsed = strtol(pos, &end, 10);
    if (end == pos) {
        return false;
    }
    *out = (int)parsed;
    return true;
}

static esp_err_t handle_tuning(const char *json)
{
    if (strstr(json, "\"params\"") == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const struct {
        const char *name;
        float fallback;
    } fields[STM32_LQR_TUNING_FLOAT_COUNT] = {
        {"Q_body_angle", 100.0f},
        {"Q_body_rate", 10.0f},
        {"Q_wheel_position", 1.0f},
        {"Q_wheel_velocity", 1.0f},
        {"R_joint_torque", 0.1f},
        {"R_wheel_torque", 0.5f},
        {"Ki_body_angle", 0.5f},
        {"max_joint_torque", 6.0f},
        {"max_wheel_torque", 3.0f},
        {"max_integral", 2.0f},
    };

    uint8_t payload[STM32_LQR_TUNING_PAYLOAD_LEN];
    uint8_t offset = 0;
    for (size_t i = 0; i < STM32_LQR_TUNING_FLOAT_COUNT; i++) {
        float value = 0.0f;
        if (!json_get_float(json, fields[i].name, fields[i].fallback, &value)) {
            return ESP_ERR_INVALID_ARG;
        }
        memcpy(&payload[offset], &value, sizeof(value));
        offset += sizeof(value);
    }

    return send_packet(PKT_TYPE_SET_TUNING, payload, offset);
}

static esp_err_t handle_mode(const char *json)
{
    char mode[24];
    if (!json_get_string(json, "mode", mode, sizeof(mode))) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t mode_id = 0;
    if (strcmp(mode, "driving") == 0) {
        mode_id = 1;
    } else if (strcmp(mode, "sitting") == 0) {
        mode_id = 2;
    } else if (strcmp(mode, "calibration") == 0) {
        mode_id = 5;
    }
    return send_packet(PKT_TYPE_SET_MODE, &mode_id, 1);
}

static esp_err_t handle_m0601c_speed(const char *json)
{
    int speed = 0;
    if (!json_get_int(json, "speed", &speed)) {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t speed_val = (int16_t)speed;
    int motor_count = 2;
    char motor[16];
    if (json_get_string(json, "motor", motor, sizeof(motor)) &&
        strcmp(motor, "left") == 0) {
        motor_count = 1;
    }

    for (int m = 0; m < motor_count; m++) {
        uint8_t payload[4] = {
            (uint8_t)m,
            0,
            (uint8_t)(speed_val >> 8),
            (uint8_t)(speed_val & 0xFF),
        };
        esp_err_t ret = send_packet(PKT_TYPE_M0601C_CMD, payload, sizeof(payload));
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

static esp_err_t handle_command(const char *json)
{
    char cmd[32];
    if (!json_get_string(json, "cmd", cmd, sizeof(cmd))) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(cmd, "enable") == 0) {
        uint8_t payload[] = {1};
        return send_packet(PKT_TYPE_SET_ENABLE, payload, sizeof(payload));
    }
    if (strcmp(cmd, "disable") == 0) {
        uint8_t payload[] = {0};
        return send_packet(PKT_TYPE_SET_ENABLE, payload, sizeof(payload));
    }
    if (strcmp(cmd, "set_mode") == 0) {
        return handle_mode(json);
    }
    if (strcmp(cmd, "m0601c_speed") == 0) {
        return handle_m0601c_speed(json);
    }

    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t handle_m0601c_cmd(const char *json)
{
    int motor_idx = 0;
    int cmd_type = 0;
    int value = 0;
    if (!json_get_int(json, "motor_idx", &motor_idx) ||
        !json_get_int(json, "cmd_type", &cmd_type) ||
        !json_get_int(json, "value", &value)) {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t v = (int16_t)value;
    uint8_t payload[4] = {
        (uint8_t)motor_idx,
        (uint8_t)cmd_type,
        (uint8_t)(v >> 8),
        (uint8_t)(v & 0xFF),
    };
    return send_packet(PKT_TYPE_M0601C_CMD, payload, sizeof(payload));
}

esp_err_t telemetry_bridge_init(const telemetry_bridge_config_t *config)
{
    if (config == NULL || config->ws_send == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_bridge, 0, sizeof(s_bridge));
    s_bridge.config = *config;
    s_bridge.status.initialized = true;
    s_bridge.status.uart_port = config->uart_port;
    return ESP_OK;
}

esp_err_t telemetry_bridge_handle_ws_text(const uint8_t *data, size_t len)
{
    if (!s_bridge.status.initialized || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    char text[256];
    if (len >= sizeof(text)) {
        s_bridge.status.command_errors++;
        s_bridge.status.last_error = "ws_command_too_large";
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(text, data, len);
    text[len] = '\0';

    char type[32];
    if (!json_get_string(text, "type", type, sizeof(type))) {
        s_bridge.status.ws_raw_commands++;
        const int written = uart_write_bytes(s_bridge.config.uart_port,
                                             (const char *)data,
                                             len);
        if (written != (int)len) {
            s_bridge.status.last_error = "uart_raw_write_failed";
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    s_bridge.status.ws_text_commands++;
    esp_err_t ret = ESP_ERR_NOT_SUPPORTED;
    if (strcmp(type, "tuning") == 0) {
        ret = handle_tuning(text);
    } else if (strcmp(type, "command") == 0) {
        ret = handle_command(text);
    } else if (strcmp(type, "m0601c_cmd") == 0) {
        ret = handle_m0601c_cmd(text);
    } else {
        ret = ESP_ERR_NOT_SUPPORTED;
    }

    if (ret != ESP_OK) {
        s_bridge.status.command_errors++;
        s_bridge.status.last_error = "command_parse_failed";
        ESP_LOGW(TAG, "WS command failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t telemetry_bridge_handle_stm32_bytes(const uint8_t *data, uint16_t len)
{
    if (!s_bridge.status.initialized || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Check for remote button packet (STM32->ESP32) */
    if (len >= 6 && data[0] == 0xAA && data[1] == 0xBB && data[2] == PKT_TYPE_REMOTE_BTN) {
        uint8_t payload_len = data[3];
        if (payload_len >= 2) {
            uint8_t buttons = data[4];
            /* Button bit0 (KEY1) pressed -> toggle light */
            if (buttons & 0x01) {
                static int light_on = 0;
                light_on = !light_on;
                light_pwm_set_mode(light_on ? LIGHT_PWM_MODE_ON : LIGHT_PWM_MODE_OFF);
            }
        }
    }

    esp_err_t ret = s_bridge.config.ws_send(data, len, true, s_bridge.config.ws_send_ctx);
    if (ret == ESP_OK) {
        s_bridge.status.stm32_packets_forwarded++;
        s_bridge.status.stm32_bytes_forwarded += len;
        s_bridge.status.last_error = NULL;
    } else {
        s_bridge.status.last_error = "ws_broadcast_failed";
    }
    return ret;
}

esp_err_t telemetry_bridge_status(telemetry_bridge_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_bridge.status;
    return ESP_OK;
}
