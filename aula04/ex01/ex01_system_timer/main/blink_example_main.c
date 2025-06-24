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
#include "driver/ledc.h"            // for PWM
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO GPIO_NUM_1

// PWM Configuration
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      (1) // Define the output GPIO
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT  // Set duty resolution to 13 bits
#define LEDC_FREQUENCY      300               // Frequency of PWM signal (300Hz)



static void configure_pwm(void){
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LEDC_OUTPUT_IO,
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


static void periodic_timer_callback(void* arg)
{
    static int duty_cycle = 0;
    duty_cycle = (duty_cycle + 10) % 110;
    set_duty_cycle(duty_cycle);
    ESP_LOGI(TAG, "Duty cycle: %d", duty_cycle);
}

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_pwm();

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "periodic"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 2000000)); // 2s


    while(1){ // keep the main task running
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };

}
