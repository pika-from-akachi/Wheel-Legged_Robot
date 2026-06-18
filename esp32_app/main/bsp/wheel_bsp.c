#include "wheel_bsp.h"

#include "driver/ledc.h"
#include "driver/spi_master.h"

#define WHEEL_STM32_UART_PORT       UART_NUM_1
#define WHEEL_STM32_UART_TX_GPIO    43
#define WHEEL_STM32_UART_RX_GPIO    44
#define WHEEL_STM32_UART_BAUD       921600
#define WHEEL_STM32_UART_BUF_SIZE   256

wheel_bsp_uart_config_t wheel_bsp_stm32_uart_config(void)
{
    return (wheel_bsp_uart_config_t) {
        .port = WHEEL_STM32_UART_PORT,
        .tx_gpio = WHEEL_STM32_UART_TX_GPIO,
        .rx_gpio = WHEEL_STM32_UART_RX_GPIO,
        .baud_rate = WHEEL_STM32_UART_BAUD,
        .rx_buffer_size = WHEEL_STM32_UART_BUF_SIZE,
    };
}

wheel_bsp_pwm_output_config_t wheel_bsp_fan_pwm_config(void)
{
    return (wheel_bsp_pwm_output_config_t) {
        .enabled = true,
        .gpio = 39,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pwm_frequency_hz = 25000,
        .duty_resolution_bits = LEDC_TIMER_10_BIT,
        .initial_duty_percent = 80,
    };
}

wheel_bsp_pwm_output_config_t wheel_bsp_light_pwm_config(void)
{
    return (wheel_bsp_pwm_output_config_t) {
        .enabled = false,
        .gpio = -1,
        .ledc_timer = LEDC_TIMER_1,
        .ledc_channel = LEDC_CHANNEL_1,
        .pwm_frequency_hz = 1000,
        .duty_resolution_bits = LEDC_TIMER_10_BIT,
        .initial_duty_percent = 0,
    };
}

wheel_bsp_screen_config_t wheel_bsp_screen_config(void)
{
    return (wheel_bsp_screen_config_t) {
        .enabled = true,
        .spi_host = SPI2_HOST,
        .pixel_clock_hz = 50 * 1000 * 1000,
        .sck_gpio = 12,
        .cs_gpio = 13,
        .data0_gpio = 48,
        .data1_gpio = 47,
        .data2_gpio = 21,
        .data3_gpio = 14,
        .reset_gpio = 35,
        .te_gpio = 36,
        .backlight_gpio = 37,
        .backlight_active_high = true,
    };
}
