#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "temp_sensor_tc74.h"
#include "driver/uart.h"
#include "esp_timer.h"

#define TC74_SDA_IO         3
#define TC74_SCL_IO         2

static uint8_t avg;
SemaphoreHandle_t xSemaphore = NULL;

void tc74_task(void* arg)
{
    i2c_master_bus_handle_t busHandle;
    i2c_master_dev_handle_t sensorHandle;
    tc74_init(&busHandle, &sensorHandle, TC74_A1_SENSOR_ADDR,
              TC74_SDA_IO, TC74_SCL_IO, TC74_SCL_DFLT_FREQ_HZ);

    const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;   // delay duration (1 second)
    BaseType_t xWasDelayed;
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount ();
    uint8_t temp;
    uint8_t last3temps[3];
    uint8_t tempIdx = 0;
    tc74_wakeup_and_read_temp(sensorHandle, &temp);
    last3temps[tempIdx++] = temp;
    while(true)
    {
        // Wait for the next cycle.
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );

        tc74_read_temp_after_temp(sensorHandle, &temp);
        last3temps[tempIdx] = temp;
        tempIdx = (tempIdx + 1) % 3;
        ESP_LOGI("task1", "Temperature: %d", temp);

        if(xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE)
        {
            avg = (last3temps[0] + last3temps[1] + last3temps[2]) / 3;
            xSemaphoreGive( xSemaphore );
        }
        
        xLastWakeTime = xTaskGetTickCount ();

    }
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

void task2(void *args){
    configure_uart();
    uint8_t data[1];
    while(true)
    {
        int length = uart_read_bytes(UART_NUM_0, data, sizeof(data), 5);
        if (length > 0) {
            if(xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE)
            {
                printf("Average temperature: %d\n", avg);
                xSemaphoreGive( xSemaphore );
            }
           
        }
        
    }
}

void app_main(void)
{
    vSemaphoreCreateBinary( xSemaphore );

    if( xSemaphore == NULL )
    {
        ESP_LOGI("main", "Failed to create semaphore");
    }

    xTaskCreate(
        tc74_task,
        "task2",
        2048,
        NULL,
        5,
        NULL
    );

    xTaskCreate(
        task2,
        "task2",
        2048,
        NULL,
        5,
        NULL
    );


    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    };

}




