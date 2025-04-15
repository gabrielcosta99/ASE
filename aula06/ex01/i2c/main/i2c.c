#include <stdio.h>
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "driver/gpio.h"
#include <string.h> 

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define SCLPIN 2
#define SDAPIN 3

#define I2C_SLAVE_NUM I2C_NUM_0
#define I2C_SLAVE_SCL_IO 21
#define I2C_SLAVE_SDA_IO 22
#define ESP_SLAVE_ADDR 0x49

i2c_master_bus_handle_t pBusHandle;
i2c_master_dev_handle_t pMasterHandle;
i2c_slave_dev_handle_t pSlaveHandle;

void config_i2c(){
    i2c_master_bus_config_t i2cMasterCfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = SCLPIN,
        .sda_io_num = SDAPIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2cMasterCfg, &pBusHandle));

    i2c_device_config_t i2cDevCfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ESP_SLAVE_ADDR,
        .scl_speed_hz = 50000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(pBusHandle, &i2cDevCfg, &pMasterHandle));

    // i2c_slave_config_t slave_cfg = {
    //     .sda_io_num = SDAPIN,
    //     .scl_io_num = SCLPIN,
    //     .slave_addr = ESP_SLAVE_ADDR,
    // };

    // ESP_ERROR_CHECK(i2c_new_slave_device(&slave_cfg, &slave_handle));
}

void send_data(){
    const char* data = "Hello world";
    size_t length = strlen(data);

    
    uint8_t *transmit_buffer = (uint8_t*)malloc(length);
    uint8_t *receive_buffer = (uint8_t*)malloc(length);
   

    // Copy data to transmit_buffer
    memcpy(transmit_buffer, data, length);
    memset(receive_buffer, 0, length);

    i2c_master_transmit(pMasterHandle, transmit_buffer, length, -1);
    i2c_slave_receive(pSlaveHandle, receive_buffer, length);

    printf("Received data: %s\n", receive_buffer);
}

void app_main(void)
{
    config_i2c();
    send_data();

}
