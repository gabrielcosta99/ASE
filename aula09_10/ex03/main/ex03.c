#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "temp_sensor_tc74.h"
#include "bme280_sensor_spi.h"
#include "lm335.c"
#include "driver/uart.h"
#include "esp_timer.h"

#define TC74_SDA_IO         3
#define TC74_SCL_IO         2

#define BME280_SDI_IO         7
#define BME280_SDO_IO         9
#define BME280_SCK_IO         6
#define BME280_CS_IO          8

typedef struct temperature_data
{
    uint8_t temperature;
    uint32_t timestamp; // add a timestamp (number of ticks)
    
}temperature_data;

static uint8_t avg;
static temperature_data min = {100, 0};
static temperature_data max = {0, 0};

static uint8_t tc74_avg;
static uint8_t bme280_avg;
static uint8_t lm335_avg;
static bool is_tc74_ready = false;
static bool is_bme280_ready = false;
static bool is_lm335_ready = false;
SemaphoreHandle_t tc74Semaphore = NULL;
SemaphoreHandle_t bme280Semaphore = NULL;
SemaphoreHandle_t lm335Semaphore = NULL;

static void tc74_task(void* arg)
{
    i2c_master_bus_handle_t busHandle;
    i2c_master_dev_handle_t sensorHandle;

    esp_err_t ret;

    tc74_init(&busHandle, &sensorHandle, TC74_A1_SENSOR_ADDR,
              TC74_SDA_IO, TC74_SCL_IO, TC74_SCL_DFLT_FREQ_HZ);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;   // delay duration (1 second)
    BaseType_t xWasDelayed;
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount ();

    uint8_t temp;
    uint8_t last3temps[3] = {0,0,0};
    uint8_t tempIdx = 0;
    uint8_t sample_count = 0;
    tc74_wakeup_and_read_temp(sensorHandle, &temp);
    last3temps[tempIdx++] = temp;
    while(true)
    {
        // Wait for the next cycle.
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );

        tc74_read_temp_after_temp(sensorHandle, &temp);
        last3temps[tempIdx] = temp;
        tempIdx = (tempIdx + 1) % 3;
        sample_count++;
        if(sample_count == 3)
        {
            is_tc74_ready = true;
        }
        
        ESP_LOGI("tc74", "Temperature: %d", temp);

        if(xSemaphoreTake(tc74Semaphore, (TickType_t) 10) == pdTRUE)
        {
            tc74_avg = (last3temps[0] + last3temps[1] + last3temps[2]) / 3;
            ESP_LOGI("tc74", "Average: %d", tc74_avg);
 
            xSemaphoreGive( tc74Semaphore );
        }
        
        xLastWakeTime = xTaskGetTickCount ();

    }
}

static void bme280_task(void* arg)
{
    spi_device_handle_t sensorHandle;
    
    // Initialize BME280 with SPI
    bme280_init(&sensorHandle, BME280_CS_IO,
               BME280_SDI_IO, BME280_SDO_IO, BME280_SCK_IO, BME280_SCL_DFLT_FREQ_HZ);
            
    // Small delay to ensure sensor is ready
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Read calibration data
    bme280_calib_data_t calib_data;
    bme280_read_calibration_data(sensorHandle, &calib_data);

    // Configure the sensor
    if (!bme280_configure(sensorHandle)) {
        printf("Failed to configure BME280 sensor\n");
        return;
    }

    printf("here\n");

    const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;   // delay duration (1 second)
    BaseType_t xWasDelayed;
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount ();

    uint8_t avg;
    uint8_t temp;
    uint8_t last3temps[3] = {0,0,0};
    uint8_t tempIdx = 0;
    uint8_t sample_count = 0;
    bme280_data_t data;
    while(true)
    {
        // Wait for the next cycle.
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );
        bme280_read_data(sensorHandle, &calib_data, &data);
        temp = data.temperature;
        last3temps[tempIdx] = temp;
        tempIdx = (tempIdx + 1) % 3;
        ESP_LOGI("bme280", "Temperature: %d", temp);
        sample_count++;
        if(sample_count == 3)
        {
            is_bme280_ready = true;
        }
        if(xSemaphoreTake(bme280Semaphore, (TickType_t) 10) == pdTRUE)
        {
            bme280_avg = (last3temps[0] + last3temps[1] + last3temps[2]) / 3;
            ESP_LOGI("bme280", "Average: %d", bme280_avg);
      
            xSemaphoreGive( bme280Semaphore );
        }
        
        xLastWakeTime = xTaskGetTickCount ();

    }
}


void lm335_task(void *args)
{
    adc_oneshot_unit_handle_t adc1_handle = NULL;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    lm335_init(&adc1_handle, &adc1_cali_chan0_handle);

    // Configure timer
    const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;   // delay duration (1 second)
    BaseType_t xWasDelayed;
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount ();

    uint8_t temp = 0;
    uint8_t last3temps[3] = {0,0,0};
    uint8_t tempIdx = 0;
    uint8_t sample_count = 0;
    int adc_raw[2][10];
    int voltage[2][10];
    while (1) {
        // Wait for the next cycle.
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );
        
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN, &adc_raw[0][0]));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
        
        float kelvin = voltage[0][0]*2/10.0;  // convert mV to Kelvin (multiply by 2 first because of the voltage divider)
        float celsius = kelvin - 273.15; // convert Kelvin to Celsius
        uint8_t temp = (uint8_t)celsius;
        last3temps[tempIdx] = temp;
        tempIdx = (tempIdx + 1) % 3;
        ESP_LOGI("lm335", "Temperature: %d", temp);
        sample_count++;
        if(sample_count == 3)
        {
            is_lm335_ready = true;
        }

        if(xSemaphoreTake(lm335Semaphore, (TickType_t) 10) == pdTRUE)
        {
            lm335_avg= (last3temps[0] + last3temps[1] + last3temps[2]) / 3;
            ESP_LOGI("lm335", "Average: %d", lm335_avg);
            
            xSemaphoreGive( lm335Semaphore );
        }
        xLastWakeTime = xTaskGetTickCount ();
        // vTaskDelay(pdMS_TO_TICKS(1000));
       
    }

    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    example_adc_calibration_deinit(adc1_cali_chan0_handle);
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
            if(xSemaphoreTake(tc74Semaphore, (TickType_t) 10) == pdTRUE  && 
               xSemaphoreTake(bme280Semaphore, (TickType_t) 10) == pdTRUE &&
               xSemaphoreTake(lm335Semaphore, (TickType_t) 10) == pdTRUE)
            {
                if(is_tc74_ready && is_bme280_ready && is_lm335_ready)
                {
                    if(data[0] == 'm')
                    {
                        printf("[timestamp: %5ld] Minimum temperature: %d\n", min.timestamp ,min.temperature);
                    }
                    else if(data[0] == 'M')
                    {
                        printf("[timestamp: %5ld] Maximum temperature: %d\n", max.timestamp ,max.temperature);
                    }
                    else if(data[0] == 'a')
                    {
                        printf("Average temperature: %d\n", avg);
                    }
                    
                }
                xSemaphoreGive( tc74Semaphore );
                xSemaphoreGive( bme280Semaphore );
                xSemaphoreGive( lm335Semaphore );
                
            }
           
        }
        
    }
}


void app_main(void)
{
    vSemaphoreCreateBinary( tc74Semaphore );
    if( tc74Semaphore == NULL )
        ESP_LOGI("main", "Failed to create tc74 semaphore");

    vSemaphoreCreateBinary( bme280Semaphore );
    if( bme280Semaphore == NULL )
        ESP_LOGI("main", "Failed to create bme280 semaphore");
    
    vSemaphoreCreateBinary( lm335Semaphore );
    if( lm335Semaphore == NULL )
        ESP_LOGI("main", "Failed to create lm335 semaphore");

    xTaskCreate(
        tc74_task,
        "tc74_task",
        4096,
        NULL,
        5,
        NULL
    );
    xTaskCreate(
        bme280_task,
        "bme280_task",
        4096,
        NULL,
        5,
        NULL
    );
    xTaskCreate(
        lm335_task,
        "lm335_task",
        4096,
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
        if(xSemaphoreTake(tc74Semaphore, (TickType_t) 10) == pdTRUE  && 
            xSemaphoreTake(bme280Semaphore, (TickType_t) 10) == pdTRUE &&
            xSemaphoreTake(lm335Semaphore, (TickType_t) 10) == pdTRUE)
        {
            if(is_tc74_ready && is_bme280_ready && is_lm335_ready)
            {
                ESP_LOGI("main", "tc74: %d, bme280: %d, lm335: %d", tc74_avg, bme280_avg, lm335_avg);
                avg = (tc74_avg + bme280_avg + lm335_avg) / 3;
                if(avg <= min.temperature)
                {
                    min.temperature = avg;
                    min.timestamp = xTaskGetTickCount();
                }
                if(avg >= max.temperature)
                {
                    max.temperature = avg;
                    max.timestamp = xTaskGetTickCount();
                }
            }
            xSemaphoreGive( tc74Semaphore );
            xSemaphoreGive( bme280Semaphore );
            xSemaphoreGive( lm335Semaphore );
        }
    };

}




