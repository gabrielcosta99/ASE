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
#include "driver/gptimer.h"
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



static volatile int latest_duty_cycle = 0;  // Shared variable

static bool IRAM_ATTR gptimer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data){
    static int duty_cycle = 0;
    duty_cycle = (duty_cycle + 10) % 110;
    set_duty_cycle(duty_cycle);
    latest_duty_cycle = duty_cycle;
    return true;
}


void config_timer(){
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));


    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));


    ESP_LOGI(TAG, "Start timer, with a period of 2 second");
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 2000000, // period = 2s
        .reload_count = 0,      // Start from 0
        .flags.auto_reload_on_alarm = true
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}



void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    // gpio_set_level(BLINK_GPIO, true);
    configure_pwm();

    config_timer();


    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // while (1) {
        //     ESP_LOGI(TAG, "Duty cycle: %d", latest_duty_cycle); // Safe to log in the main loop
        //     vTaskDelay(pdMS_TO_TICKS(2000)); // Print every second
        // }
    };
    // int dutycycle = 80; // 20%
    

    // while (1) {

    //     set_duty_cycle(dutycycle);
    // }
}
