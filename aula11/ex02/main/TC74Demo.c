#include <stdio.h>
#include "esp_err.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "temp_sensor_tc74.h"
#include "esp_sleep.h"

#define TC74_SDA_IO         3
#define TC74_SCL_IO         2

#define TEMP_MIN            10
#define TEMP_MAX            35
  
#define LED_OUTPUT_IO       1
#define DUTY_CYCLE_RES      LEDC_TIMER_10_BIT

#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_CHANNEL        LEDC_CHANNEL_0

#define TIMER_WAKEUP_TIME_US  3*1000*1000

void led_init(int ledPin, uint16_t dutyCycleRes)
{
    ledc_timer_config_t ledcTimerCfg = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = dutyCycleRes,
        .freq_hz          = 1000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledcTimerCfg));

    ledc_channel_config_t ledcChannelCfg = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = ledPin,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledcChannelCfg));
}

void set_led_duty_cycle(uint8_t tempMin, uint8_t tempMax,
                        uint8_t currentTemp, uint16_t dutyCycleRes) 
{
    uint16_t dutyCycle = (((1 << dutyCycleRes) - 1) * (currentTemp - tempMin)) /
                                                      (tempMax - tempMin);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, dutyCycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void app_main(void)
{
    i2c_master_bus_handle_t busHandle;
    i2c_master_dev_handle_t sensorHandle;
    uint8_t temperature;
    
    led_init(LED_OUTPUT_IO, DUTY_CYCLE_RES);
    
    tc74_init(&busHandle, &sensorHandle, TC74_A1_SENSOR_ADDR,
              TC74_SDA_IO, TC74_SCL_IO, TC74_SCL_DFLT_FREQ_HZ);

    tc74_wakeup(sensorHandle);
    
    tc74_read_temp_after_cfg(sensorHandle, &temperature);
    esp_sleep_enable_timer_wakeup(TIMER_WAKEUP_TIME_US);
  
    while ((temperature >= TEMP_MIN) && (temperature <= TEMP_MAX))
    {
        printf("\nTemperature: %d ºC", temperature);
        set_led_duty_cycle(TEMP_MIN, TEMP_MAX, temperature, DUTY_CYCLE_RES);

        esp_light_sleep_start(); 
        // vTaskDelay(3000 / portTICK_PERIOD_MS);
        tc74_read_temp_after_temp(sensorHandle, &temperature);
    }
 
    printf("\nError - Temperature out of range: %d ºC\n", temperature);
    
    tc74_free(busHandle, sensorHandle);
}
