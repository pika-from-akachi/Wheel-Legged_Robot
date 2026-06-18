#include "screen_player.h"

#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "screen_roboeyes.h"

#define ST77916_CMD_SWRESET 0x01
#define ST77916_CMD_CASET   0x2A
#define ST77916_CMD_RASET   0x2B
#define ST77916_CMD_RAMWR   0x2C

#define ST77916_SPI_WRITE_CMD 0x02
#define ST77916_QSPI_WRITE 0x32
#define ST77916_CMD_BUFFER_BYTES 64
#define ST77916_SPI_MAX_TRANSFER_BYTES (16 * 1024)
#define SCREEN_FILL_ROWS 20
#define SCREEN_TICK_TASK_STACK 4096
#define SCREEN_TICK_TASK_PRIORITY 5
#define SCREEN_TICK_TASK_CORE 1
#define SCREEN_TICK_INTERVAL_MS 2
#define SCREEN_ROBOEYES_ASSET "roboeyes_normal"

typedef struct {
    uint8_t cmd;
    uint8_t data[14];
    uint8_t data_len;
    uint16_t delay_ms;
} lcd_init_cmd_t;

static const char *TAG = "SCREEN_PLAYER";

static screen_player_status_t s_status;
static screen_player_config_t s_config;
static spi_device_handle_t s_lcd_spi;
static uint8_t *s_spi_cmd_buffer;
static uint8_t *s_spi_tx_buffer;
static TaskHandle_t s_tick_task;

static const lcd_init_cmd_t s_boe_st77916_init[] = {
    {0xF0, {0x28}, 1, 0},
    {0xF2, {0x28}, 1, 0},
    {0x73, {0xF0}, 1, 0},
    {0x76, {0x0F}, 1, 0},
    {0x7C, {0xD1}, 1, 0},
    {0x83, {0xE0}, 1, 0},
    {0x84, {0x61}, 1, 0},
    {0xF2, {0x82}, 1, 0},
    {0xF0, {0x00}, 1, 0},
    {0xF0, {0x01}, 1, 0},
    {0xF1, {0x01}, 1, 0},
    {0xB0, {0x52}, 1, 0},
    {0xB1, {0x49}, 1, 0},
    {0xB2, {0x24}, 1, 0},
    {0xB3, {0x01}, 1, 0},
    {0xB4, {0x66}, 1, 0},
    {0xB5, {0x44}, 1, 0},
    {0xB6, {0xC5}, 1, 0},
    {0xB7, {0x40}, 1, 0},
    {0xB8, {0x86}, 1, 0},
    {0xB9, {0x15}, 1, 0},
    {0xBA, {0x00}, 1, 0},
    {0xBB, {0x08}, 1, 0},
    {0xBC, {0x08}, 1, 0},
    {0xBD, {0x00}, 1, 0},
    {0xBE, {0x00}, 1, 0},
    {0xBF, {0x07}, 1, 0},
    {0xC0, {0x80}, 1, 0},
    {0xC1, {0x10}, 1, 0},
    {0xC2, {0x37}, 1, 0},
    {0xC3, {0x80}, 1, 0},
    {0xC4, {0x10}, 1, 0},
    {0xC5, {0x37}, 1, 0},
    {0xC6, {0xA9}, 1, 0},
    {0xC7, {0x41}, 1, 0},
    {0xC8, {0x01}, 1, 0},
    {0xC9, {0xA9}, 1, 0},
    {0xCA, {0x41}, 1, 0},
    {0xCB, {0x01}, 1, 0},
    {0xCC, {0x7F}, 1, 0},
    {0xCD, {0x7F}, 1, 0},
    {0xCE, {0xFF}, 1, 0},
    {0xD0, {0x91}, 1, 0},
    {0xD1, {0x68}, 1, 0},
    {0xD2, {0x68}, 1, 0},
    {0xF5, {0x00, 0xA5}, 2, 0},
    {0xF1, {0x10}, 1, 0},
    {0xF0, {0x00}, 1, 0},
    {0xF0, {0x02}, 1, 0},
    {0xE0, {0xF0, 0x0E, 0x14, 0x0B, 0x0B, 0x16, 0x3A, 0x44, 0x4E, 0x18, 0x14, 0x13, 0x2F, 0x35}, 14, 0},
    {0xE1, {0xF0, 0x0D, 0x13, 0x0B, 0x0A, 0x16, 0x39, 0x43, 0x4E, 0x17, 0x13, 0x13, 0x2E, 0x34}, 14, 0},
    {0xF0, {0x10}, 1, 0},
    {0xF3, {0x10}, 1, 0},
    {0xE0, {0x09}, 1, 0},
    {0xE1, {0x00}, 1, 0},
    {0xE2, {0x03}, 1, 0},
    {0xE3, {0x00}, 1, 0},
    {0xE4, {0xE0}, 1, 0},
    {0xE5, {0x06}, 1, 0},
    {0xE6, {0x21}, 1, 0},
    {0xE7, {0x00}, 1, 0},
    {0xE8, {0x05}, 1, 0},
    {0xE9, {0x82}, 1, 0},
    {0xEA, {0xDE}, 1, 0},
    {0xEB, {0xC0}, 1, 0},
    {0xEC, {0x40}, 1, 0},
    {0xED, {0x84}, 1, 0},
    {0xEE, {0xFF}, 1, 0},
    {0xEF, {0x71}, 1, 0},
    {0xF8, {0xFF}, 1, 0},
    {0xF9, {0x50}, 1, 0},
    {0xFA, {0xFF}, 1, 0},
    {0xFB, {0xF3}, 1, 0},
    {0xFC, {0x00}, 1, 0},
    {0xFD, {0x00}, 1, 0},
    {0xFE, {0x00}, 1, 0},
    {0xFF, {0x00}, 1, 0},
    {0x60, {0x42}, 1, 0},
    {0x61, {0xDF}, 1, 0},
    {0x62, {0x40}, 1, 0},
    {0x63, {0x40}, 1, 0},
    {0x64, {0x02}, 1, 0},
    {0x65, {0x00}, 1, 0},
    {0x66, {0x00}, 1, 0},
    {0x67, {0x00}, 1, 0},
    {0x68, {0x00}, 1, 0},
    {0x69, {0x00}, 1, 0},
    {0x6A, {0x00}, 1, 0},
    {0x6B, {0x00}, 1, 0},
    {0x70, {0x42}, 1, 0},
    {0x71, {0xDF}, 1, 0},
    {0x72, {0x40}, 1, 0},
    {0x73, {0x40}, 1, 0},
    {0x74, {0x01}, 1, 0},
    {0x75, {0x00}, 1, 0},
    {0x76, {0x00}, 1, 0},
    {0x77, {0x00}, 1, 0},
    {0x78, {0x00}, 1, 0},
    {0x79, {0x00}, 1, 0},
    {0x7A, {0x00}, 1, 0},
    {0x7B, {0x00}, 1, 0},
    {0x80, {0x48}, 1, 0},
    {0x81, {0x00}, 1, 0},
    {0x82, {0x04}, 1, 0},
    {0x83, {0x02}, 1, 0},
    {0x84, {0xDC}, 1, 0},
    {0x85, {0x00}, 1, 0},
    {0x86, {0x00}, 1, 0},
    {0x87, {0x00}, 1, 0},
    {0x88, {0x48}, 1, 0},
    {0x89, {0x00}, 1, 0},
    {0x8A, {0x06}, 1, 0},
    {0x8B, {0x02}, 1, 0},
    {0x8C, {0xDE}, 1, 0},
    {0x8D, {0x00}, 1, 0},
    {0x8E, {0x00}, 1, 0},
    {0x8F, {0x00}, 1, 0},
    {0x90, {0x48}, 1, 0},
    {0x91, {0x00}, 1, 0},
    {0x92, {0x08}, 1, 0},
    {0x93, {0x02}, 1, 0},
    {0x94, {0xE0}, 1, 0},
    {0x95, {0x00}, 1, 0},
    {0x96, {0x00}, 1, 0},
    {0x97, {0x00}, 1, 0},
    {0x98, {0x48}, 1, 0},
    {0x99, {0x00}, 1, 0},
    {0x9A, {0x0A}, 1, 0},
    {0x9B, {0x02}, 1, 0},
    {0x9C, {0xE2}, 1, 0},
    {0x9D, {0x00}, 1, 0},
    {0x9E, {0x00}, 1, 0},
    {0x9F, {0x00}, 1, 0},
    {0xA0, {0x48}, 1, 0},
    {0xA1, {0x00}, 1, 0},
    {0xA2, {0x03}, 1, 0},
    {0xA3, {0x02}, 1, 0},
    {0xA4, {0xDB}, 1, 0},
    {0xA5, {0x00}, 1, 0},
    {0xA6, {0x00}, 1, 0},
    {0xA7, {0x00}, 1, 0},
    {0xA8, {0x48}, 1, 0},
    {0xA9, {0x00}, 1, 0},
    {0xAA, {0x05}, 1, 0},
    {0xAB, {0x02}, 1, 0},
    {0xAC, {0xDD}, 1, 0},
    {0xAD, {0x00}, 1, 0},
    {0xAE, {0x00}, 1, 0},
    {0xAF, {0x00}, 1, 0},
    {0xB0, {0x48}, 1, 0},
    {0xB1, {0x00}, 1, 0},
    {0xB2, {0x07}, 1, 0},
    {0xB3, {0x02}, 1, 0},
    {0xB4, {0xDF}, 1, 0},
    {0xB5, {0x00}, 1, 0},
    {0xB6, {0x00}, 1, 0},
    {0xB7, {0x00}, 1, 0},
    {0xB8, {0x48}, 1, 0},
    {0xB9, {0x00}, 1, 0},
    {0xBA, {0x09}, 1, 0},
    {0xBB, {0x02}, 1, 0},
    {0xBC, {0xE1}, 1, 0},
    {0xBD, {0x00}, 1, 0},
    {0xBE, {0x00}, 1, 0},
    {0xBF, {0x00}, 1, 0},
    {0xC0, {0x65}, 1, 0},
    {0xC1, {0x74}, 1, 0},
    {0xC2, {0x47}, 1, 0},
    {0xC3, {0x56}, 1, 0},
    {0xC4, {0xAA}, 1, 0},
    {0xC5, {0x11}, 1, 0},
    {0xC6, {0x00}, 1, 0},
    {0xC7, {0x2A}, 1, 0},
    {0xC8, {0xA2}, 1, 0},
    {0xC9, {0x33}, 1, 0},
    {0xD0, {0x65}, 1, 0},
    {0xD1, {0x74}, 1, 0},
    {0xD2, {0x47}, 1, 0},
    {0xD3, {0x56}, 1, 0},
    {0xD4, {0xAA}, 1, 0},
    {0xD5, {0x11}, 1, 0},
    {0xD6, {0x00}, 1, 0},
    {0xD7, {0x2A}, 1, 0},
    {0xD8, {0xA2}, 1, 0},
    {0xD9, {0x33}, 1, 0},
    {0xF3, {0x01}, 1, 0},
    {0xF0, {0x00}, 1, 0},
    {0x21, {0}, 0, 0},
    {0x11, {0}, 0, 120},
    {0x29, {0}, 0, 0},
    {0x3A, {0x55}, 1, 0},
};

static esp_err_t lcd_ensure_cmd_buffer(void)
{
    if (s_spi_cmd_buffer != NULL) {
        return ESP_OK;
    }

    s_spi_cmd_buffer = heap_caps_malloc(ST77916_CMD_BUFFER_BYTES,
                                        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    return s_spi_cmd_buffer == NULL ? ESP_ERR_NO_MEM : ESP_OK;
}

static esp_err_t lcd_spi_write_command(uint8_t cmd, const void *data, size_t len)
{
    if (s_lcd_spi == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL && len > 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len + 4U > ST77916_CMD_BUFFER_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_RETURN_ON_ERROR(lcd_ensure_cmd_buffer(), TAG, "lcd cmd buffer alloc failed");

    s_spi_cmd_buffer[0] = ST77916_SPI_WRITE_CMD;
    s_spi_cmd_buffer[1] = 0x00;
    s_spi_cmd_buffer[2] = cmd;
    s_spi_cmd_buffer[3] = 0x00;
    if (len > 0) {
        memcpy(&s_spi_cmd_buffer[4], data, len);
    }

    spi_transaction_t trans = {
        .length = (len + 4U) * 8U,
        .tx_buffer = s_spi_cmd_buffer,
    };
    return spi_device_polling_transmit(s_lcd_spi, &trans);
}

static esp_err_t lcd_qspi_write_pixels(uint8_t cmd, const void *data, size_t len)
{
    if (s_lcd_spi == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL && len > 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool data_in_psram = data != NULL && esp_ptr_external_ram(data);
    if (data_in_psram && s_spi_tx_buffer == NULL) {
        s_spi_tx_buffer = heap_caps_malloc(ST77916_SPI_MAX_TRANSFER_BYTES,
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (s_spi_tx_buffer == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(s_lcd_spi, portMAX_DELAY), TAG, "acquire lcd spi bus failed");

    size_t offset = 0;
    bool first = true;
    esp_err_t ret = ESP_OK;

    do {
        size_t chunk_size = len - offset;
        if (chunk_size > ST77916_SPI_MAX_TRANSFER_BYTES) {
            chunk_size = ST77916_SPI_MAX_TRANSFER_BYTES;
        }

        const uint8_t *chunk = chunk_size > 0 ? (const uint8_t *)data + offset : NULL;
        if (data_in_psram && chunk_size > 0) {
            memcpy(s_spi_tx_buffer, chunk, chunk_size);
            chunk = s_spi_tx_buffer;
        }

        spi_transaction_ext_t trans = {0};
        trans.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
        trans.command_bits = first ? 8 : 0;
        trans.address_bits = first ? 24 : 0;
        trans.base.cmd = ST77916_QSPI_WRITE;
        trans.base.addr = ((uint32_t)cmd << 8);
        trans.base.length = chunk_size * 8U;
        trans.base.tx_buffer = chunk;

        if (chunk_size > 0) {
            trans.base.flags |= SPI_TRANS_MODE_QIO;
        }

        const bool last = (offset + chunk_size) >= len;
        if (!last) {
            trans.base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
        }

        ret = spi_device_polling_transmit(s_lcd_spi, &trans.base);
        if (ret != ESP_OK) {
            break;
        }

        offset += chunk_size;
        first = false;
    } while (offset < len);

    spi_device_release_bus(s_lcd_spi);
    return ret;
}

static esp_err_t lcd_tx_param(uint8_t cmd, const void *param, size_t len)
{
    return lcd_spi_write_command(cmd, param, len);
}

static esp_err_t lcd_tx_cmd(uint8_t cmd)
{
    return lcd_tx_param(cmd, NULL, 0);
}

static esp_err_t lcd_run_init_sequence(void)
{
    for (size_t i = 0; i < sizeof(s_boe_st77916_init) / sizeof(s_boe_st77916_init[0]); ++i) {
        const lcd_init_cmd_t *entry = &s_boe_st77916_init[i];
        ESP_RETURN_ON_ERROR(lcd_tx_param(entry->cmd, entry->data, entry->data_len),
                            TAG,
                            "BOE ST77916 init cmd 0x%02x failed",
                            entry->cmd);
        if (entry->delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(entry->delay_ms));
        }
    }
    return ESP_OK;
}

static esp_err_t lcd_set_window(int x, int y, int width, int height)
{
    const uint16_t x0 = (uint16_t)x;
    const uint16_t y0 = (uint16_t)y;
    const uint16_t x1 = (uint16_t)(x + width - 1);
    const uint16_t y1 = (uint16_t)(y + height - 1);
    const uint8_t caset[] = {
        (uint8_t)(x0 >> 8), (uint8_t)x0,
        (uint8_t)(x1 >> 8), (uint8_t)x1,
    };
    const uint8_t raset[] = {
        (uint8_t)(y0 >> 8), (uint8_t)y0,
        (uint8_t)(y1 >> 8), (uint8_t)y1,
    };

    ESP_RETURN_ON_ERROR(lcd_tx_param(ST77916_CMD_CASET, caset, sizeof(caset)), TAG, "CASET failed");
    ESP_RETURN_ON_ERROR(lcd_tx_param(ST77916_CMD_RASET, raset, sizeof(raset)), TAG, "RASET failed");
    return ESP_OK;
}

static esp_err_t lcd_reset_panel(void)
{
    if (s_config.reset_gpio < 0) {
        return lcd_tx_cmd(ST77916_CMD_SWRESET);
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << s_config.reset_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "reset gpio config failed");
    gpio_set_level(s_config.reset_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(s_config.reset_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(s_config.reset_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static esp_err_t lcd_backlight_set(bool on)
{
    if (s_config.backlight_gpio < 0) {
        return ESP_OK;
    }

    const int level = (on == s_config.backlight_active_high) ? 1 : 0;
    gpio_set_level(s_config.backlight_gpio, level);
    return ESP_OK;
}

static esp_err_t lcd_init_backlight(void)
{
    if (s_config.backlight_gpio < 0) {
        return ESP_OK;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << s_config.backlight_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "backlight gpio config failed");
    return lcd_backlight_set(true);
}

static esp_err_t lcd_init_panel(void)
{
    ESP_RETURN_ON_ERROR(lcd_init_backlight(), TAG, "backlight init failed");
    ESP_RETURN_ON_ERROR(lcd_reset_panel(), TAG, "panel reset failed");
    ESP_RETURN_ON_ERROR(lcd_backlight_set(true), TAG, "backlight on failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_RETURN_ON_ERROR(lcd_run_init_sequence(), TAG, "BOE ST77916 panel init failed");
    ESP_RETURN_ON_ERROR(lcd_set_window(0, 0, SCREEN_PLAYER_WIDTH, SCREEN_PLAYER_HEIGHT), TAG, "set full window failed");
    return ESP_OK;
}

esp_err_t screen_player_draw_rgb565_rect(int x, int y, int width, int height, const void *rgb565, size_t len)
{
    if (!s_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (rgb565 == NULL || width <= 0 || height <= 0 ||
        x < 0 || y < 0 ||
        x + width > SCREEN_PLAYER_WIDTH ||
        y + height > SCREEN_PLAYER_HEIGHT ||
        len != (size_t)width * (size_t)height * 2U) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(lcd_set_window(x, y, width, height), TAG, "set draw window failed");
    return lcd_qspi_write_pixels(ST77916_CMD_RAMWR, rgb565, len);
}

esp_err_t screen_player_fill_rgb565(uint16_t rgb565)
{
    if (!s_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const int rows_per_chunk = SCREEN_FILL_ROWS;
    const size_t chunk_bytes = SCREEN_PLAYER_WIDTH * rows_per_chunk * 2U;
    uint8_t *chunk = heap_caps_malloc(chunk_bytes, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (chunk == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const uint8_t hi = (uint8_t)(rgb565 >> 8);
    const uint8_t lo = (uint8_t)rgb565;
    for (size_t i = 0; i < chunk_bytes; i += 2) {
        chunk[i] = hi;
        chunk[i + 1] = lo;
    }

    esp_err_t ret = ESP_OK;
    for (int y = 0; y < SCREEN_PLAYER_HEIGHT; y += rows_per_chunk) {
        const int rows = (SCREEN_PLAYER_HEIGHT - y) < rows_per_chunk
                             ? (SCREEN_PLAYER_HEIGHT - y)
                             : rows_per_chunk;
        ret = screen_player_draw_rgb565_rect(0,
                                             y,
                                             SCREEN_PLAYER_WIDTH,
                                             rows,
                                             chunk,
                                             SCREEN_PLAYER_WIDTH * rows * 2U);
        if (ret != ESP_OK) {
            break;
        }
    }

    free(chunk);
    return ret;
}

void screen_player_report_frame(esp_err_t ret, uint32_t elapsed_us, const char *error)
{
    s_status.last_frame_us = elapsed_us;
    if (ret == ESP_OK) {
        s_status.frames_rendered++;
        s_status.last_error = NULL;
        return;
    }

    s_status.frames_dropped++;
    s_status.last_error = error == NULL ? "screen_frame_failed" : error;
}

static void screen_player_tick_task(void *arg)
{
    (void)arg;
    TickType_t wake_time = xTaskGetTickCount();
    const TickType_t interval_ticks =
        pdMS_TO_TICKS(SCREEN_TICK_INTERVAL_MS) > 0 ? pdMS_TO_TICKS(SCREEN_TICK_INTERVAL_MS) : 1;

    while (true) {
        (void)screen_player_tick();
        vTaskDelayUntil(&wake_time, interval_ticks);
    }
}

static esp_err_t screen_player_start_tick_task(void)
{
    if (s_tick_task != NULL) {
        return ESP_OK;
    }

    const BaseType_t ok = xTaskCreatePinnedToCore(screen_player_tick_task,
                                                  "screen_tick",
                                                  SCREEN_TICK_TASK_STACK,
                                                  NULL,
                                                  SCREEN_TICK_TASK_PRIORITY,
                                                  &s_tick_task,
                                                  SCREEN_TICK_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t screen_player_init(const screen_player_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;
    s_status = (screen_player_status_t) {
        .enabled = config->enabled,
        .initialized = false,
        .last_error = config->enabled ? "not_initialized" : "disabled",
    };
    s_tick_task = NULL;

    if (!config->enabled) {
        return ESP_OK;
    }

    spi_bus_config_t bus_cfg = {
        .data0_io_num = config->data0_gpio,
        .data1_io_num = config->data1_gpio,
        .sclk_io_num = config->sck_gpio,
        .data2_io_num = config->data2_gpio,
        .data3_io_num = config->data3_gpio,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .data_io_default_level = false,
        .max_transfer_sz = ST77916_SPI_MAX_TRANSFER_BYTES,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
    };
    esp_err_t ret = spi_bus_initialize((spi_host_device_t)config->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        s_status.last_error = "spi_bus_init_failed";
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .mode = 0,
        .clock_speed_hz = config->pixel_clock_hz,
        .spics_io_num = config->cs_gpio,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device((spi_host_device_t)config->spi_host,
                                           &dev_cfg,
                                           &s_lcd_spi),
                        TAG,
                        "add lcd spi device failed");
    ESP_RETURN_ON_ERROR(lcd_init_panel(), TAG, "panel init failed");

    s_status.initialized = true;
    s_status.last_error = NULL;
    return ESP_OK;
}

esp_err_t screen_player_start_eyes(void)
{
    if (!s_status.enabled) {
        return ESP_OK;
    }
    if (!s_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_status.playing) {
        return ESP_OK;
    }

    esp_err_t ret = screen_player_fill_rgb565(0x0000);
    if (ret != ESP_OK) {
        s_status.last_error = "screen_clear_failed";
        return ret;
    }

    ret = screen_roboeyes_begin_expression("normal");
    if (ret != ESP_OK) {
        s_status.last_error = "roboeyes_begin_failed";
        return ret;
    }

    s_status.playing = true;
    s_status.active_asset = SCREEN_ROBOEYES_ASSET;
    s_status.last_error = NULL;
    ret = screen_player_start_tick_task();
    if (ret != ESP_OK) {
        s_status.playing = false;
        s_status.last_error = "screen_tick_task_failed";
        return ret;
    }

    ESP_LOGI(TAG, "FluxGarage RoboEyes animation active: %dx%d QSPI",
             SCREEN_PLAYER_WIDTH,
             SCREEN_PLAYER_HEIGHT);
    return ESP_OK;
}

esp_err_t screen_player_tick(void)
{
    if (!s_status.enabled || !s_status.initialized || !s_status.playing) {
        return ESP_OK;
    }

    esp_err_t ret = screen_roboeyes_update();
    if (ret != ESP_OK) {
        screen_player_report_frame(ret, 0, "roboeyes_update_failed");
    }
    return ret;
}

esp_err_t screen_player_status(screen_player_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_status;
    return ESP_OK;
}
