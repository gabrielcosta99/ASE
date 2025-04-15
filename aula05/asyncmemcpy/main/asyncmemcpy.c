#include <stdio.h>
#include "esp_timer.h"
#include "esp_async_memcpy.h"

#define BUFFER_SIZE  4096
#define REPEAT_COUNT 10000

static uint32_t srcBuffer[BUFFER_SIZE];
static uint32_t dstBuffer[BUFFER_SIZE];

static bool AsyncMemcpyCallback(async_memcpy_handle_t handle, async_memcpy_event_t *event, void* args)
{
   uint32_t* pFlag = (uint32_t*)args;
   *pFlag = 1;
   return true;
}

void app_main(void)
{
   async_memcpy_config_t config = ASYNC_MEMCPY_DEFAULT_CONFIG();
   config.backlog = 8;
   async_memcpy_handle_t driver = NULL;
   esp_async_memcpy_install(&config, &driver);
  
   for (int i = 0; i < BUFFER_SIZE; i++)
   {
      srcBuffer[i] = i;
   }
   
   int64_t startTime = esp_timer_get_time();
   
   for (int j = 0; j < REPEAT_COUNT; j++)
   {
      uint32_t flag = 0;
      esp_async_memcpy(driver, dstBuffer, srcBuffer, BUFFER_SIZE * sizeof(uint32_t),
                       AsyncMemcpyCallback, &flag);
      while (!flag);
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
