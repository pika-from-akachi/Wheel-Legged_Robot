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
#define ST77916_CMD_SLPOUT  0x11
#define ST77916_CMD_NORON   0x13
#define ST77916_CMD_INVOFF  0x20
#define ST77916_CMD_DISPON  0x29
#define ST77916_CMD_CASET   0x2A
#define ST77916_CMD_RASET   0x2B
#define ST77916_CMD_RAMWR   0x2C
#define ST77916_CMD_TEOFF   0x34
#define ST77916_CMD_MADCTL  0x36
#define ST77916_CMD_IDMOFF  0x38
#define ST77916_CMD_COLMOD  0x3A
#define ST77916_CMD_WRDISBV 0x51
#define ST77916_CMD_WRCTRLD 0x53
#define ST77916_CMD_RESSET1 0xD0
#define ST77916_CMD_RESSET2 0xD1
#define ST77916_CMD_RESSET3 0xD2

#define ST77916_QSPI_WRITE 0x32
#define ST77916_SPI_MAX_TRANSFER_BYTES (16 * 1024)
#define SCREEN_FILL_ROWS 20
#define SCREEN_TASK_STACK 12288
#define SCREEN_TASK_PRIORITY 4
#define SCREEN_BOOT_SETTLE_MS 650
#define SCREEN_ROBOEYES_ASSET "roboeyes_normal"

static const char *TAG = "SCREEN_PLAYER";

static screen_player_status_t s_status;
static screen_player_config_t s_config;
static spi_device_handle_t s_lcd_spi;
static uint8_t *s_spi_tx_buffer;
static TaskHandle_t s_player_task;

static esp_err_t lcd_qspi_write(uint8_t cmd, const void *data, size_t len)
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
    return lcd_qspi_write(cmd, param, len);
}

static esp_err_t lcd_tx_cmd(uint8_t cmd)
{
    return lcd_tx_param(cmd, NULL, 0);
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
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(s_config.reset_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
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
    return lcd_backlight_set(false);
}

static esp_err_t lcd_init_panel(void)
{
    ESP_RETURN_ON_ERROR(lcd_init_backlight(), TAG, "backlight init failed");
    ESP_RETURN_ON_ERROR(lcd_reset_panel(), TAG, "panel reset failed");

    const uint8_t resset1 = 0x91;
    const uint8_t resset2 = 0x68;
    const uint8_t resset3 = 0x86;
    const uint8_t colmod = 0x05;
    const uint8_t madctl = 0x00;
    const uint8_t brightness = 0xFF;
    const uint8_t ctrl_display = 0x2C;

    ESP_RETURN_ON_ERROR(lcd_tx_cmd(ST77916_CMD_SWRESET), TAG, "software reset failed");
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(lcd_tx_param(ST77916_CMD_RESSET1, &resset1, 1), TAG, "RESSET1 failed");
    ESP_RETURN_ON_ERROR(lcd_tx_param(ST77916_CMD_RESSET2, &resset2, 1), TAG, "RESSET2 failed");
    ESP_RETURN_ON_ERROR(lcd_tx_param(ST77916_CMD_RESSET3, &resset3, 1), TAG, "RESSET3 failed");
    ESP_RETURN_ON_ERROR(lcd_tx_param(ST77916_CMD_MADCTL, &madctl, 1), TAG, "MADCTL failed");
    ESP_RETURN_ON_ERROR(lcd_tx_param(ST77916_CMD_COLMOD, &colmod, 1), TAG, "COLMOD failed");
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(ST77916_CMD_IDMOFF), TAG, "idle off failed");
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(ST77916_CMD_TEOFF), TAG, "TE off failed");
    ESP_RETURN_ON_ERROR(lcd_tx_param(ST77916_CMD_WRDISBV, &brightness, 1), TAG, "brightness failed");
    ESP_RETURN_ON_ERROR(lcd_tx_param(ST77916_CMD_WRCTRLD, &ctrl_display, 1), TAG, "display ctrl failed");
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(ST77916_CMD_INVOFF), TAG, "invert off failed");
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(ST77916_CMD_SLPOUT), TAG, "sleep out failed");
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(ST77916_CMD_NORON), TAG, "normal on failed");
    ESP_RETURN_ON_ERROR(lcd_set_window(0, 0, SCREEN_PLAYER_WIDTH, SCREEN_PLAYER_HEIGHT), TAG, "set full window failed");
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(ST77916_CMD_DISPON), TAG, "display on failed");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(lcd_backlight_set(true), TAG, "backlight on failed");
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
    return lcd_qspi_write(ST77916_CMD_RAMWR, rgb565, len);
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

static void player_task(void *arg)
{
    (void)arg;

    s_status.playing = true;
    s_status.active_asset = "startup_settle";
    s_status.last_error = NULL;
    vTaskDelay(pdMS_TO_TICKS(SCREEN_BOOT_SETTLE_MS));

    s_status.active_asset = "startup_reveal";
    esp_err_t ret = screen_roboeyes_play_startup();
    if (ret != ESP_OK) {
        s_status.last_error = "startup_reveal_failed";
        ESP_LOGW(TAG, "startup reveal failed: %s", esp_err_to_name(ret));
    }

    s_status.active_asset = SCREEN_ROBOEYES_ASSET;
    ret = screen_roboeyes_begin_expression("normal");
    if (ret != ESP_OK) {
        s_status.last_error = "roboeyes_begin_failed";
        ESP_LOGW(TAG, "RoboEyes begin failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "RoboEyes animation active: %dx%d QSPI",
             SCREEN_PLAYER_WIDTH,
             SCREEN_PLAYER_HEIGHT);

    while (1) {
        ret = screen_roboeyes_update();
        if (ret != ESP_OK) {
            screen_player_report_frame(ret, 0, "roboeyes_update_failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
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
    if (s_player_task != NULL) {
        return ESP_OK;
    }

    s_status.playing = true;
    s_status.active_asset = "startup_settle";
    if (xTaskCreatePinnedToCore(player_task,
                                "screen_roboeyes",
                                SCREEN_TASK_STACK,
                                NULL,
                                SCREEN_TASK_PRIORITY,
                                &s_player_task,
                                1) != pdPASS) {
        s_status.last_error = "task_create_failed";
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t screen_player_status(screen_player_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_status;
    return ESP_OK;
}
