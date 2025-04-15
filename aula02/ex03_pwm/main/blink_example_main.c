/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/ledc.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

// PWM Configuration
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT  // Set duty resolution (0-8191)
#define LEDC_FREQUENCY      5000               // Frequency of PWM signal (5kHz)

// static uint8_t s_led_state = 0;

// static void blink_led(void)
// {
//     /* Set the GPIO level according to the state (LOW or HIGH)*/
//     gpio_set_level(BLINK_GPIO, s_led_state);
// }

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void configure_pwm(void){
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQUENCY,
        .duty_resolution = LEDC_DUTY_RES,
        .clk_cfg         = LEDC_AUTO_CLK
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num   = BLINK_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };

    ledc_channel_config(&ledc_channel);
}
static void set_duty_cycle(int duty_cycle)
{
    int duty = (duty_cycle * ((1 << LEDC_DUTY_RES) - 1)) / 100;  // Convert % to raw duty
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}


void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    // gpio_set_level(BLINK_GPIO, true);
    configure_pwm();
    int dutycycle = 10; 
    set_duty_cycle(dutycycle);

}
