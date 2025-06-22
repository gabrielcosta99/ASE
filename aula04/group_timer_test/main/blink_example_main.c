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
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/ledc.h"            // for PWM
#include "driver/gptimer.h"
#include "esp_log.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
static volatile bool timer_fired = false;

static bool IRAM_ATTR gptimer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data){
    timer_fired = true;
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


    ESP_LOGI(TAG, "Start timer, with a period of 1 second");
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 1000000, // period = 1s
        .reload_count = 0,      // Start from 0
        .flags.auto_reload_on_alarm = true
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}


void app_main(void)
{

   config_timer();
    

    while (1) {
        if (timer_fired) {
            ESP_LOGI(TAG, "Timer alarmed!");
            timer_fired = false;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay to prevent busy waiting
    }

    
}
