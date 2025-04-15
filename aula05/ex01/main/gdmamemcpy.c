#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "soc/gdma_reg.h"
#include "soc/system_reg.h"
#include "esp_heap_caps.h"


#define BUFFER_SIZE  4096
#define REPEAT_COUNT 10000

#define DMA_FRAME_SIZE 2048

#define ADDR_MASK 0xFFFFF
#define FLAGS_MASK 0xFFF00000

static uint32_t srcBuffer[BUFFER_SIZE];
static uint32_t dstBuffer[BUFFER_SIZE];

typedef struct gdma_descriptor gdma_descriptor_t;

// Complete structure definition
struct gdma_descriptor {
   uint32_t size:12;      // Size of transfer (max: 4096 bytes per descriptor)
   uint32_t length:12;    // Actual length of data
   uint32_t eof:1;        // End of Frame flag (1 if last descriptor)
   uint32_t owner:1;      // 1 = owned by hardware, 0 = owned by software
   uint8_t *buffer;       // Source data with proper type
   gdma_descriptor_t *next; // Pointer to next descriptor (NULL if last)
};


gdma_descriptor_t *setup_dma_desc(uint8_t *buffer, size_t length) {
   // Calculate number of descriptors required
   int num_desc = (length + DMA_FRAME_SIZE - 1) / DMA_FRAME_SIZE; 

   // Allocate descriptors dynamically
   gdma_descriptor_t *desc = (gdma_descriptor_t *)heap_caps_malloc(num_desc * sizeof(gdma_descriptor_t), MALLOC_CAP_DMA);
   if (!desc) {
       ESP_LOGE("DMA", "Failed to allocate DMA descriptors!");
       return NULL;
   }

   // Initialize descriptors
   for (int i = 0; i < num_desc; i++) {
       desc[i].buffer = (buffer + i * DMA_FRAME_SIZE);
       desc[i].size = (length > DMA_FRAME_SIZE) ? DMA_FRAME_SIZE : length;
       desc[i].length = desc[i].size;
       desc[i].owner = 1;
       desc[i].eof = (i == num_desc - 1) ? 1 : 0;
       desc[i].next = (i == num_desc - 1) ? NULL : &desc[i + 1];
       length -= desc[i].size;
   }

   return desc;
}

void init_gdma()
{
  REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
  REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
}

void gdma_mem_to_mem_cpy(const void *src, void *dst, size_t length) {

   // reset the state machine of GDMA’s transmit channel and FIFO pointer
   REG_SET_BIT(GDMA_OUT_CONF0_CH0_REG,GDMA_OUT_RST_CH0); 
   REG_CLR_BIT(GDMA_OUT_CONF0_CH0_REG,GDMA_OUT_RST_CH0);

   // set GDMA_IN_RST_CHn first to 1 and then to 0, to reset the state machine of GDMA’s receive channel and FIFO pointer;
   REG_SET_BIT(GDMA_IN_CONF0_CH0_REG,GDMA_IN_RST_CH0); 
   REG_CLR_BIT(GDMA_IN_CONF0_CH0_REG,GDMA_IN_RST_CH0);

   // Load an outlink, and configure GDMA_OUTLINK_ADDR_CH0 with address of the first transmit descriptor
   gdma_descriptor_t *transmiter_descriptor = setup_dma_desc((uint8_t *)src, length);

   // Load an inlink, and configure GDMA_INLINK_ADDR_CHn with address of the first receive descriptor;
   gdma_descriptor_t *receiver_descriptor = setup_dma_desc((uint8_t *)dst, length);

   // Check if descriptors were allocated
   if (!transmiter_descriptor || !receiver_descriptor) {
      ESP_LOGE("DMA", "Failed to allocate DMA descriptors.");
      return;
  }

   // Set up DMA outlink address
   uint32_t transmiter_flags = REG_READ(GDMA_OUT_LINK_CH0_REG) & FLAGS_MASK;
   uint32_t transmiter_addr = ((uint32_t)&transmiter_descriptor[0]) & ADDR_MASK;
   REG_WRITE(GDMA_OUT_LINK_CH0_REG, transmiter_flags | transmiter_addr);

   // Set up DMA inlink address
   uint32_t receiver_flags = REG_READ(GDMA_IN_LINK_CH0_REG) & FLAGS_MASK;
   uint32_t receiver_addr = ((uint32_t)&receiver_descriptor[0]) & ADDR_MASK;
   REG_WRITE(GDMA_IN_LINK_CH0_REG, receiver_flags | receiver_addr);


   // Configure memory-to-memory transfer
   REG_CLR_BIT(GDMA_OUT_PERI_SEL_CH0_REG, GDMA_PERI_OUT_SEL_CH0); 
   REG_CLR_BIT(GDMA_IN_PERI_SEL_CH0_REG, GDMA_PERI_IN_SEL_CH0);   
   REG_SET_BIT(GDMA_IN_CONF0_CH0_REG,GDMA_MEM_TRANS_EN_CH0); 

   // Start DMA transfer
   REG_SET_BIT(GDMA_OUT_LINK_CH0_REG,GDMA_OUTLINK_START_CH0);
   REG_SET_BIT(GDMA_IN_LINK_CH0_REG,GDMA_INLINK_START_CH0);

   // Polling for transfer completion by checking the EOF flag
   while (!(REG_READ(GDMA_INT_RAW_CH0_REG) & GDMA_OUT_TOTAL_EOF_CH0_INT_RAW));

   // Clear the interrupt flag
   REG_SET_BIT(GDMA_INT_CLR_CH0_REG, GDMA_IN_SUC_EOF_CH0_INT_CLR);

   // Free the descriptors
   free(transmiter_descriptor);
   free(receiver_descriptor);

}


void app_main(void)
{
   
   for (int i = 0; i < BUFFER_SIZE; i++)
   {
      srcBuffer[i] = i;
   }
   init_gdma();
   int64_t startTime = esp_timer_get_time();
   for (int j = 0; j < REPEAT_COUNT; j++)
   {
      gdma_mem_to_mem_cpy(srcBuffer, dstBuffer, BUFFER_SIZE * sizeof(uint32_t));
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
