#ifndef FAN_PWM_H
#define FAN_PWM_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "wheel_bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef wheel_bsp_pwm_output_config_t fan_pwm_config_t;

typedef struct {
    bool enabled;
    bool initialized;
    int gpio;
    int ledc_timer;
    int ledc_channel;
    uint32_t pwm_frequency_hz;
    uint8_t duty_resolution_bits;
    uint8_t duty_percent;
    const char *last_error;
} fan_pwm_status_t;

esp_err_t fan_pwm_init(const fan_pwm_config_t *config);
void fan_pwm_tick(void);
esp_err_t fan_pwm_status(fan_pwm_status_t *out);
esp_err_t fan_pwm_set_duty(uint8_t percent);

#ifdef __cplusplus
}
#endif

#endif /* FAN_PWM_H */
