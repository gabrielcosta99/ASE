#include <stdio.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"

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

static void ConfigureUART(void)
{
   uart_config_t uartConfig =
   {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
   };
   uart_param_config(UART_NUM_0, &uartConfig);
   uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL, 0);
}

void app_main(void)
{
   volatile uint32_t workerCnt = 0;
   const char dot[2] = ".\n";
   
   uint8_t prevSwVal = 0;
   uint8_t currSwVal;
   
   ConfigureUART();
   ConfigureGPIOs();
   
   int64_t startTime = esp_timer_get_time();

   while (1)
   {
      workerCnt++;
      
      if (workerCnt % INTERMEDIATE_COUNT == 0)
      {
         uart_write_bytes(UART_NUM_0, dot, sizeof(dot));
      }
         
      if (workerCnt == FINAL_COUNT)
      {
         int64_t endTime = esp_timer_get_time();
         
         printf("Elapsed time: %lld microseconds\n", (endTime - startTime));
         break;
      }
      
      currSwVal = ReadSw();
      if ((!prevSwVal) && (currSwVal))
      {
         ToggleLed();
      }
      prevSwVal = currSwVal;
    }
}
