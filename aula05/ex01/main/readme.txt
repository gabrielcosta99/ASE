    Starting with the initial setup, I created a function called "init_gdma", responsible for activating 
the DMA clock and handling its reset/deactivation.
    Next, I implemented "gdma_mem_to_mem_cpy", a function that uses DMA to copy data from one memory location 
(srcBuffer) to another (dstBuffer).
    In order for this to work, I defined a structure called "gdma_descriptor", which serves as the actual data unit 
for transmission and reception.
    Finally, I created "setup_dma_desc", a function that instantiates a "gdma_descriptor" based on the buffer we want to 
send and it's size.

    All of this was successfully tested, with the final elapsed time of 258 milliseconds.