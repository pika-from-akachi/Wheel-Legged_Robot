#ifndef LIGHT_PWM_H
#define LIGHT_PWM_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "wheel_bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef wheel_bsp_pwm_output_config_t light_pwm_config_t;

typedef enum {
    LIGHT_PWM_MODE_OFF = 0,
    LIGHT_PWM_MODE_ON,
    LIGHT_PWM_MODE_BREATH,
    LIGHT_PWM_MODE_FLASH,
} light_pwm_mode_t;

typedef struct {
    bool enabled;
    bool initialized;
    int gpio;
    int ledc_timer;
    int ledc_channel;
    uint32_t pwm_frequency_hz;
    uint8_t duty_resolution_bits;
    uint8_t duty_percent;
    light_pwm_mode_t mode;
    const char *last_error;
} light_pwm_status_t;

esp_err_t light_pwm_init(const light_pwm_config_t *config);
void light_pwm_tick(void);
esp_err_t light_pwm_status(light_pwm_status_t *out);
esp_err_t light_pwm_set_duty(uint8_t channel, uint8_t percent);
esp_err_t light_pwm_set_mode(light_pwm_mode_t mode);
const char *light_pwm_mode_name(light_pwm_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* LIGHT_PWM_H */
