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

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define INPUT_GPIO GPIO_NUM_3
static uint8_t s_led_state = 0;

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

// Since GPIO 3 is an input, if we want it to be default ON (HIGH) when not connected, we should:
//     Enable pull-up resistor (GPIO_PULLUP_ONLY) in our code.
//     When connected to GND, it will read LOW (0).
//     When disconnected or externally pulled HIGH, it will read HIGH (1).
// If we want the opposite behavior (default OFF (LOW)), we use GPIO_PULLDOWN_ONLY.
static void configure_gpio_input(void)
{
    ESP_LOGI(TAG, "Configuring input on GPIO %d", INPUT_GPIO);
    gpio_reset_pin(INPUT_GPIO);
    /* Set the GPIO as a push/pull input */
    gpio_set_direction(INPUT_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO, GPIO_PULLDOWN_ONLY);
}


void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led();
    configure_gpio_input();
    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");

        blink_led();
        /* Toggle the LED state */
        // s_led_state = !s_led_state;
        s_led_state = gpio_get_level(INPUT_GPIO);   // to turn on the LED, connect GPIO 3 to "+" (3V3). To turn off the LED, connect GPIO 3 to GND.
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
