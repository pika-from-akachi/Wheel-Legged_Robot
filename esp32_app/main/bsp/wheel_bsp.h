#ifndef WHEEL_BSP_H
#define WHEEL_BSP_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uart_port_t port;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;
    int rx_buffer_size;
} wheel_bsp_uart_config_t;

typedef struct {
    bool enabled;
    int gpio;
    int ledc_timer;
    int ledc_channel;
    uint32_t pwm_frequency_hz;
    uint8_t duty_resolution_bits;
    uint8_t initial_duty_percent;
} wheel_bsp_pwm_output_config_t;

typedef struct {
    bool enabled;
    int spi_host;
    int pixel_clock_hz;
    int sck_gpio;
    int cs_gpio;
    int data0_gpio;
    int data1_gpio;
    int data2_gpio;
    int data3_gpio;
    int reset_gpio;
    int te_gpio;
    int backlight_gpio;
    bool backlight_active_high;
} wheel_bsp_screen_config_t;

wheel_bsp_uart_config_t wheel_bsp_stm32_uart_config(void);
wheel_bsp_pwm_output_config_t wheel_bsp_fan_pwm_config(void);
wheel_bsp_pwm_output_config_t wheel_bsp_light_pwm_config(void);
wheel_bsp_screen_config_t wheel_bsp_screen_config(void);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_BSP_H */
