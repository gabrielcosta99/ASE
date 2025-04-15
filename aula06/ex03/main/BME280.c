#include <stdio.h>
#include "esp_err.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "bme280_sensor_i2c.h"

#define BME280_SDA_IO         3
#define BME280_SCL_IO         2


void app_main(void)
{
    i2c_master_bus_handle_t busHandle;
    i2c_master_dev_handle_t sensorHandle;
    bme280_init(&busHandle, &sensorHandle,BME280_SLAVE_I2C_ADDR,
               BME280_SDA_IO, BME280_SCL_IO, BME280_SCL_DFLT_FREQ_HZ);
            
    bme280_calib_data_t calib_data;
    bme280_read_calibration_data(sensorHandle, &calib_data);

    bme280_configure(sensorHandle);

    bme280_data_t data;
    while (1)
    {
        bme280_read_data(sensorHandle, &calib_data, &data);
        // printf("\nTemperature: %f ºC", data.temperature);
        printf("\nTemperature: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f %%", 
            data.temperature, data.pressure, data.humidity);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

