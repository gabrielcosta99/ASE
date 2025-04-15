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
#include "driver/uart.h"


static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO              0
#define CONFIG_BLINK_PERIOD_MS     100


static void blink_led(int s_led_state)
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

static void configure_uart(void){
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
}

void app_main(void)
{
    uint8_t s_led_state = 0;

    uint8_t data[1];  // Buffer to store user input

    /* Configure the peripheral according to the LED type */
    configure_led();
    configure_uart();
    while (1) {
        // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");

        // blink_led(s_led_state);
        // /* Toggle the LED state */
        // s_led_state = !s_led_state;
        
        // printf("input: %d\n", input);
        vTaskDelay(CONFIG_BLINK_PERIOD_MS / portTICK_PERIOD_MS);
        // int length = uart_read_bytes(UART_NUM_0, data, sizeof(data), 2000 / portTICK_PERIOD_MS);
        int length = uart_read_bytes(UART_NUM_0, data, sizeof(data), 0);
        if (length > 0) {
            uart_write_bytes(UART_NUM_0, data, sizeof(data));
        }
    }
}
