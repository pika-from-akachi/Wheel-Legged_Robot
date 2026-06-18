#include "screen_player.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "esp32s3/rom/tjpgd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ST77916_CMD_SWRESET 0x01
#define ST77916_CMD_SLPOUT  0x11
#define ST77916_CMD_NORON   0x13
#define ST77916_CMD_INVON   0x21
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
#define ST77916_SPI_MAX_TRANSFER_BYTES (SCREEN_PLAYER_WIDTH * 80 * 2)
#define SCREEN_FILL_ROWS 20

#define SCREEN_VIDEO_MAGIC 0x314A5657u /* WVJ1 */
#define SCREEN_VIDEO_HEADER_SIZE 32u
#define SCREEN_VIDEO_FRAME_HEADER_SIZE 8u
#define SCREEN_VIDEO_MAX_JPEG_BYTES (220 * 1024)
#define SCREEN_VIDEO_TJPG_WORK_BYTES (32 * 1024)
#define SCREEN_VIDEO_TASK_STACK 9216
#define SCREEN_VIDEO_TASK_PRIORITY 4

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint16_t width;
    uint16_t height;
    uint16_t fps;
    uint16_t flags;
    uint32_t frame_count;
    uint32_t frame_table_offset;
    uint32_t data_offset;
    uint32_t reserved;
} screen_video_header_t;

typedef struct {
    const uint8_t *jpeg_data;
    uint32_t jpeg_size;
    uint32_t offset;
    uint32_t bytes_left;
    uint8_t *framebuffer;
    uint32_t stride_bytes;
} jpeg_decode_context_t;

static const char *TAG = "SCREEN_PLAYER";
static screen_player_status_t s_status;
static screen_player_config_t s_config;
static spi_device_handle_t s_lcd_spi;
static uint8_t *s_framebuffer;
static uint8_t *s_jpeg_buffer;
static uint8_t *s_tjpg_work;
static TaskHandle_t s_player_task;
static char s_boot_path[64];
static char s_loop_path[64];

static esp_err_t mount_spiffs_if_needed(void);

static esp_err_t ensure_video_resources(void)
{
    esp_err_t ret = mount_spiffs_if_needed();
    if (ret != ESP_OK) {
        s_status.last_error = "spiffs_mount_failed";
        return ret;
    }

    if (s_framebuffer == NULL) {
        s_framebuffer = heap_caps_malloc(SCREEN_PLAYER_WIDTH * SCREEN_PLAYER_HEIGHT * 2U,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_framebuffer == NULL) {
            s_framebuffer = heap_caps_malloc(SCREEN_PLAYER_WIDTH * SCREEN_PLAYER_HEIGHT * 2U,
                                             MALLOC_CAP_8BIT);
        }
    }
    if (s_tjpg_work == NULL) {
        s_tjpg_work = heap_caps_malloc(SCREEN_VIDEO_TJPG_WORK_BYTES, MALLOC_CAP_8BIT);
    }
    if (s_jpeg_buffer == NULL) {
        s_jpeg_buffer = heap_caps_malloc(SCREEN_VIDEO_MAX_JPEG_BYTES,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_jpeg_buffer == NULL) {
            s_jpeg_buffer = heap_caps_malloc(SCREEN_VIDEO_MAX_JPEG_BYTES,
                                             MALLOC_CAP_8BIT);
        }
    }

    if (s_framebuffer == NULL || s_jpeg_buffer == NULL || s_tjpg_work == NULL) {
        s_status.last_error = "no_memory";
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static bool read_exact(FILE *file, void *out, size_t len)
{
    return fread(out, 1, len, file) == len;
}

static esp_err_t mount_spiffs_if_needed(void)
{
    if (esp_spiffs_mounted(NULL)) {
        return ESP_OK;
    }

    const esp_vfs_spiffs_conf_t conf = {
        .base_path = "/www",
        .partition_label = NULL,
        .max_files = 8,
        .format_if_mount_failed = false,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret == ESP_ERR_INVALID_STATE && esp_spiffs_mounted(NULL)) {
        return ESP_OK;
    }
    return ret;
}

static esp_err_t lcd_qspi_write(uint8_t cmd, const void *data, size_t len)
{
    if (s_lcd_spi == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL && len > 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(s_lcd_spi, portMAX_DELAY), TAG, "acquire lcd spi bus failed");

    size_t offset = 0;
    bool first = true;
    const bool data_in_psram = data != NULL && esp_ptr_external_ram(data);
    esp_err_t ret = ESP_OK;

    do {
        size_t chunk_size = len - offset;
        if (chunk_size > ST77916_SPI_MAX_TRANSFER_BYTES) {
            chunk_size = ST77916_SPI_MAX_TRANSFER_BYTES;
        }

        spi_transaction_ext_t trans = {0};
        trans.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
        trans.command_bits = first ? 8 : 0;
        trans.address_bits = first ? 24 : 0;
        trans.base.cmd = ST77916_QSPI_WRITE;
        trans.base.addr = ((uint32_t)cmd << 8);
        trans.base.length = chunk_size * 8U;
        trans.base.tx_buffer = chunk_size > 0 ? (const uint8_t *)data + offset : NULL;

        if (chunk_size > 0) {
            trans.base.flags |= SPI_TRANS_MODE_QIO;
            if (data_in_psram) {
                trans.base.flags |= SPI_TRANS_DMA_USE_PSRAM;
            }
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
    return lcd_backlight_set(true);
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
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(ST77916_CMD_INVON), TAG, "invert on failed");
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

static UINT jpeg_input_func(JDEC *jd, BYTE *buff, UINT nbyte)
{
    jpeg_decode_context_t *input = (jpeg_decode_context_t *)jd->device;
    UINT to_read = nbyte;
    if (to_read > input->bytes_left) {
        to_read = input->bytes_left;
    }

    if (buff == NULL) {
        input->offset += to_read;
    } else {
        memcpy(buff, input->jpeg_data + input->offset, to_read);
        input->offset += to_read;
    }
    input->bytes_left -= to_read;
    return to_read;
}

static UINT jpeg_output_func(JDEC *jd, void *bitmap, JRECT *rect)
{
    jpeg_decode_context_t *out = (jpeg_decode_context_t *)jd->device;
    const int rect_w = (int)rect->right - (int)rect->left + 1;
    const int rect_h = (int)rect->bottom - (int)rect->top + 1;
    const uint8_t *src = (const uint8_t *)bitmap;

    if (rect_w <= 0 || rect_h <= 0 ||
        rect->right >= SCREEN_PLAYER_WIDTH ||
        rect->bottom >= SCREEN_PLAYER_HEIGHT) {
        return 0;
    }

    for (int row = 0; row < rect_h; row++) {
        uint8_t *dst = out->framebuffer +
                       ((uint32_t)(rect->top + row) * out->stride_bytes) +
                       ((uint32_t)rect->left * 2U);
        const uint8_t *line = src + ((uint32_t)row * (uint32_t)rect_w * 3U);
        for (int col = 0; col < rect_w; col++) {
            const uint8_t r = line[(col * 3) + 0];
            const uint8_t g = line[(col * 3) + 1];
            const uint8_t b = line[(col * 3) + 2];
            const uint16_t rgb565 = ((uint16_t)(r & 0xF8) << 8) |
                                    ((uint16_t)(g & 0xFC) << 3) |
                                    ((uint16_t)b >> 3);
            dst[(col * 2) + 0] = (uint8_t)(rgb565 >> 8);
            dst[(col * 2) + 1] = (uint8_t)rgb565;
        }
    }
    return 1;
}

static bool read_video_header(FILE *file, screen_video_header_t *header)
{
    uint8_t raw[SCREEN_VIDEO_HEADER_SIZE];
    if (!read_exact(file, raw, sizeof(raw))) {
        return false;
    }

    *header = (screen_video_header_t) {
        .magic = read_le32(&raw[0]),
        .version = read_le16(&raw[4]),
        .header_size = read_le16(&raw[6]),
        .width = read_le16(&raw[8]),
        .height = read_le16(&raw[10]),
        .fps = read_le16(&raw[12]),
        .flags = read_le16(&raw[14]),
        .frame_count = read_le32(&raw[16]),
        .frame_table_offset = read_le32(&raw[20]),
        .data_offset = read_le32(&raw[24]),
        .reserved = read_le32(&raw[28]),
    };

    return header->magic == SCREEN_VIDEO_MAGIC &&
           header->version == 1 &&
           header->header_size == SCREEN_VIDEO_HEADER_SIZE &&
           header->width == SCREEN_PLAYER_WIDTH &&
           header->height == SCREEN_PLAYER_HEIGHT &&
           header->fps > 0 &&
           header->frame_count > 0;
}

static esp_err_t decode_and_draw_frame(FILE *file, uint32_t jpeg_size)
{
    if (jpeg_size == 0 || jpeg_size > SCREEN_VIDEO_MAX_JPEG_BYTES) {
        s_status.last_error = "jpeg_frame_too_large";
        return ESP_ERR_INVALID_SIZE;
    }
    if (!read_exact(file, s_jpeg_buffer, jpeg_size)) {
        s_status.last_error = "jpeg_frame_read_failed";
        return ESP_FAIL;
    }

    jpeg_decode_context_t decode = {
        .jpeg_data = s_jpeg_buffer,
        .jpeg_size = jpeg_size,
        .offset = 0,
        .bytes_left = jpeg_size,
        .framebuffer = s_framebuffer,
        .stride_bytes = SCREEN_PLAYER_WIDTH * 2U,
    };
    JDEC jd = {0};

    JRESULT jret = jd_prepare(&jd, jpeg_input_func, s_tjpg_work, SCREEN_VIDEO_TJPG_WORK_BYTES, &decode);
    if (jret != JDR_OK) {
        s_status.decode_errors++;
        s_status.last_error = "jpeg_prepare_failed";
        return ESP_FAIL;
    }
    if (jd.width != SCREEN_PLAYER_WIDTH || jd.height != SCREEN_PLAYER_HEIGHT) {
        s_status.decode_errors++;
        s_status.last_error = "jpeg_size_mismatch";
        return ESP_ERR_INVALID_SIZE;
    }

    jret = jd_decomp(&jd, jpeg_output_func, 0);
    if (jret != JDR_OK) {
        s_status.decode_errors++;
        s_status.last_error = "jpeg_decode_failed";
        return ESP_FAIL;
    }

    return screen_player_draw_rgb565_rect(0,
                                          0,
                                          SCREEN_PLAYER_WIDTH,
                                          SCREEN_PLAYER_HEIGHT,
                                          s_framebuffer,
                                          SCREEN_PLAYER_WIDTH * SCREEN_PLAYER_HEIGHT * 2U);
}

static esp_err_t play_video_file(const char *path, bool loop_forever)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        ESP_LOGW(TAG, "video open failed: %s errno=%d", path, errno);
        s_status.last_error = "video_open_failed";
        return ESP_FAIL;
    }

    screen_video_header_t header;
    if (!read_video_header(file, &header)) {
        fclose(file);
        s_status.last_error = "video_header_invalid";
        return ESP_ERR_INVALID_RESPONSE;
    }

    const int64_t frame_period_us = 1000000LL / header.fps;
    ESP_LOGI(TAG, "Playing %s: %lux%u @ %u fps, %lu frames",
             path,
             (unsigned long)header.width,
             header.height,
             header.fps,
             (unsigned long)header.frame_count);

    do {
        if (fseek(file, (long)header.data_offset, SEEK_SET) != 0) {
            s_status.last_error = "video_seek_failed";
            fclose(file);
            return ESP_FAIL;
        }

        int64_t next_frame_us = esp_timer_get_time();
        for (uint32_t frame = 0; frame < header.frame_count; frame++) {
            uint8_t raw[SCREEN_VIDEO_FRAME_HEADER_SIZE];
            if (!read_exact(file, raw, sizeof(raw))) {
                s_status.last_error = "frame_header_read_failed";
                fclose(file);
                return ESP_FAIL;
            }

            const uint32_t jpeg_size = read_le32(&raw[0]);
            const uint32_t duration_us = read_le32(&raw[4]);
            const int64_t start_us = esp_timer_get_time();
            esp_err_t ret = decode_and_draw_frame(file, jpeg_size);
            const int64_t elapsed_us = esp_timer_get_time() - start_us;
            s_status.last_frame_us = (uint32_t)elapsed_us;
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "frame %lu failed: %s", (unsigned long)frame, esp_err_to_name(ret));
                fclose(file);
                return ret;
            }
            s_status.frames_rendered++;
            s_status.last_error = NULL;

            next_frame_us += duration_us != 0 ? duration_us : frame_period_us;
            int64_t sleep_us = next_frame_us - esp_timer_get_time();
            if (sleep_us > 1000) {
                vTaskDelay(pdMS_TO_TICKS((uint32_t)(sleep_us / 1000)));
            } else if (sleep_us < -frame_period_us) {
                s_status.frames_dropped++;
                next_frame_us = esp_timer_get_time();
            } else {
                taskYIELD();
            }
        }
    } while (loop_forever);

    fclose(file);
    return ESP_OK;
}

static void player_task(void *arg)
{
    (void)arg;
    s_status.playing = true;
    s_status.active_asset = s_boot_path;

    esp_err_t ret = play_video_file(s_boot_path, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "boot video skipped: %s", esp_err_to_name(ret));
    }

    s_status.active_asset = s_loop_path;
    while (1) {
        ret = play_video_file(s_loop_path, true);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "loop video failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
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

esp_err_t screen_player_start_sequence(const char *boot_path, const char *loop_path)
{
    if (!s_status.enabled) {
        return ESP_OK;
    }
    if (!s_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (boot_path == NULL || loop_path == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_player_task != NULL) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(ensure_video_resources(), TAG, "video resources init failed");

    strlcpy(s_boot_path, boot_path, sizeof(s_boot_path));
    strlcpy(s_loop_path, loop_path, sizeof(s_loop_path));
    s_status.playing = true;
    s_status.active_asset = s_boot_path;
    if (xTaskCreate(player_task,
                    "screen_player",
                    SCREEN_VIDEO_TASK_STACK,
                    NULL,
                    SCREEN_VIDEO_TASK_PRIORITY,
                    &s_player_task) != pdPASS) {
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
