#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


void task1(void *args){
    
}



void app_main(void)
{

    xTaskCreate(
        task1,
        "task1",
        2048,
        NULL,
        5,
        NULL
    );

}
