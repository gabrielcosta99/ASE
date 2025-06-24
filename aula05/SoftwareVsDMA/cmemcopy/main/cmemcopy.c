#include <stdio.h>
#include <string.h>
#include "esp_timer.h"

#define BUFFER_SIZE  4096
#define REPEAT_COUNT 10000

static uint32_t srcBuffer[BUFFER_SIZE];
static uint32_t dstBuffer[BUFFER_SIZE];
   
void app_main(void)
{
   for (int i = 0; i < BUFFER_SIZE; i++)
   {
      srcBuffer[i] = i;
   }
   
   int64_t startTime = esp_timer_get_time();
   
   for (int j = 0; j < REPEAT_COUNT; j++)
   {
      memcpy(dstBuffer, srcBuffer, BUFFER_SIZE * sizeof(uint32_t));
   }
   
   int64_t endTime = esp_timer_get_time();
         
   printf("Elapsed time: %lld microseconds\n", (endTime - startTime));
   
   for (int i = 0; i < BUFFER_SIZE; i++)
   {
      if (dstBuffer[i] != i)
      {
         printf("Destination buffer verification failed!\n");
         return;
      }
   }
   
   printf("Destination buffer verification succeeded!\n");
}
