#include "fan_pwm.h"

#include "driver/ledc.h"

static fan_pwm_config_t s_config;
static fan_pwm_status_t s_status;

static uint32_t duty_from_percent(uint8_t percent)
{
    const uint32_t max_duty = (1UL << s_config.duty_resolution_bits) - 1UL;
    return (max_duty * percent) / 100U;
}

esp_err_t fan_pwm_init(const fan_pwm_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;
    s_status = (fan_pwm_status_t) {
        .enabled = config->enabled,
        .initialized = false,
        .gpio = config->gpio,
        .ledc_timer = config->ledc_timer,
        .ledc_channel = config->ledc_channel,
        .pwm_frequency_hz = config->pwm_frequency_hz,
        .duty_resolution_bits = config->duty_resolution_bits,
        .duty_percent = 0,
        .last_error = config->enabled ? NULL : "disabled",
    };

    if (!config->enabled) {
        return ESP_OK;
    }
    if (config->gpio < 0 || config->pwm_frequency_hz == 0 ||
        config->duty_resolution_bits == 0 || config->duty_resolution_bits > 20) {
        s_status.last_error = "invalid_config";
        return ESP_ERR_INVALID_ARG;
    }

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)config->duty_resolution_bits,
        .timer_num = (ledc_timer_t)config->ledc_timer,
        .freq_hz = config->pwm_frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer);
    if (ret != ESP_OK) {
        s_status.last_error = "ledc_timer_config_failed";
        return ret;
    }

    ledc_channel_config_t channel = {
        .gpio_num = config->gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)config->ledc_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = (ledc_timer_t)config->ledc_timer,
        .duty = duty_from_percent(config->initial_duty_percent),
        .hpoint = 0,
    };
    ret = ledc_channel_config(&channel);
    if (ret != ESP_OK) {
        s_status.last_error = "ledc_channel_config_failed";
        return ret;
    }

    s_status.initialized = true;
    s_status.duty_percent = config->initial_duty_percent;
    s_status.last_error = NULL;
    return ESP_OK;
}

void fan_pwm_tick(void)
{
}

esp_err_t fan_pwm_status(fan_pwm_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_status;
    return ESP_OK;
}

esp_err_t fan_pwm_set_duty(uint8_t percent)
{
    if (percent > 100) {
        s_status.last_error = "invalid_duty";
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_status.enabled) {
        s_status.last_error = "disabled";
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_status.initialized) {
        s_status.last_error = "uninitialized";
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  (ledc_channel_t)s_config.ledc_channel,
                                  duty_from_percent(percent));
    if (ret == ESP_OK) {
        ret = ledc_update_duty(LEDC_LOW_SPEED_MODE,
                               (ledc_channel_t)s_config.ledc_channel);
    }
    if (ret != ESP_OK) {
        s_status.last_error = "ledc_write_failed";
        return ret;
    }

    s_status.duty_percent = percent;
    s_status.last_error = NULL;
    return ESP_OK;
}
