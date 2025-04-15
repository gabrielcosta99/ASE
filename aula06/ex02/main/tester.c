#include <stdio.h>
#include "esp_err.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "temp_sensor_tc74.h"

#define TC74_SDA_IO         3
#define TC74_SCL_IO         2


void app_main(void)
{
    i2c_master_bus_handle_t busHandle;
    i2c_master_dev_handle_t sensorHandle;
    tc74_init(&busHandle, &sensorHandle, TC74_A1_SENSOR_ADDR,
              TC74_SDA_IO, TC74_SCL_IO, TC74_SCL_DFLT_FREQ_HZ);
    // tc74_wakeup(sensorHandle);
    // tc74_read_temp_after_cfg(sensorHandle, &temp);

    uint8_t temp;
    tc74_wakeup_and_read_temp(sensorHandle, &temp);
    while (1)
    {
        tc74_read_temp_after_temp(sensorHandle, &temp);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
