#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <string.h>
#include "esp_log.h"

#define TAG "SPI_EXAMPLE"

#define PIN_NUM_MISO 4
#define PIN_NUM_MOSI 5
#define PIN_NUM_SCLK 6  // Add SCLK pin definition

spi_device_handle_t devHandle;

void configure_spi() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_SCLK,  // Add SCLK pin
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    
    spi_device_interface_config_t spiDevCfg = {
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .mode = 0,
        .spics_io_num = -1,  // If not using chip select, set to -1
        .queue_size = 1,
        .clock_speed_hz = 50000,
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &spiDevCfg, &devHandle);
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "SPI configured successfully");
}

void send_data() {
    const char* data = "Hello world";
    size_t length = strlen(data);
    
    // Allocate buffers for transmission and reception
    uint8_t* tx_buffer = (uint8_t*)malloc(length);
    uint8_t* rx_buffer = (uint8_t*)malloc(length);
    
    // Copy data to tx_buffer
    memcpy(tx_buffer, data, length);
    memset(rx_buffer, 0, length);  
    
    spi_transaction_t transCfg = {
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
        .length = length * 8,  // Length in bits (each char is 8 bits)
        .rxlength = length * 8,
    };
    
    esp_err_t ret = spi_device_polling_transmit(devHandle, &transCfg);
    if (ret == ESP_OK) {
        printf("Transmitted: %s\n", data);
        printf("Received: %s\n", rx_buffer);
    } else {
        ESP_LOGE(TAG, "SPI transmission failed");
    }
    
    free(tx_buffer);
    free(rx_buffer);
}

void app_main(void) {
    configure_spi();
    send_data();
}