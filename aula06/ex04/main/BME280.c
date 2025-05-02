#include <stdio.h>
#include "esp_err.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "bme280_sensor_spi.h"

#define BME280_SDI_IO         6
#define BME280_SDO_IO         9
#define BME280_SCK_IO         7
#define BME280_CS_IO          8


void app_main(void)
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
    
    // Allow sensor to stabilize after configuration
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Read and display data
    bme280_data_t data;
    while (1)
    {
        bme280_read_data(sensorHandle, &calib_data, &data);
        printf("\nTemperature: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f %%", 
            data.temperature, data.pressure / 100.0f, data.humidity);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Longer delay for better readability
    }
}

