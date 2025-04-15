#include <stdio.h>
#include "driver/uart.h"
#include <string.h>
#include "esp_log.h"

const uart_port_t uart_num = UART_NUM_1;

void configure_uart(){
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
}

void install_uart_driver(){
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    // QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, \
                                            0,0, NULL,  0));
    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 4, 5, UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
}

int sendData(const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI("TX_TASK", "Wrote %d bytes", txBytes);
    return txBytes;
}

static void receive_data(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(1024 + 1);
    const int rxBytes = uart_read_bytes(UART_NUM_1, data, 1024, 1000 / portTICK_PERIOD_MS);
    if (rxBytes > 0) {
        data[rxBytes] = 0;
        printf("Read %d bytes: '%s'\n", rxBytes, data);
    }
    free(data);
}


void app_main(void)
{
    // DONT FORGET TO CONNECT GPIO 4 AND 5
    configure_uart();
    install_uart_driver();

    sendData("Hello world");
    receive_data(NULL);
}
