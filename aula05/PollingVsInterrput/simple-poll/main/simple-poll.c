#include <stdio.h>
#include "driver/gpio.h"

#define GPIO_SW   0 
#define GPIO_LED  1

#define INTERMEDIATE_COUNT 1000000
#define FINAL_COUNT        50000000

static void ConfigureGPIOs(void)
{
   gpio_reset_pin(GPIO_SW);
   gpio_set_direction(GPIO_SW, GPIO_MODE_INPUT);
    
   gpio_reset_pin(GPIO_LED);
   gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
}

uint8_t ReadSw(void)
{
   return gpio_get_level(GPIO_SW);
}

static void ToggleLed(void)
{
   static uint8_t ledState = 0;
   ledState = !ledState;
   gpio_set_level(GPIO_LED, ledState);
}  

void app_main(void)
{  
 
   
   ConfigureGPIOs();
   
   while (1)
   {
      while(!ReadSw());
      ToggleLed();
      while(ReadSw());

      
   }
}
