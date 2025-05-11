#include "bme280_sensor_spi.h"
#include <string.h>





void bme280_init(spi_device_handle_t* pSensorHandle,
                int csPin, int sdiPin, int sdoPin, int sckPin, uint32_t clkSpeedHz)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = sdoPin,
        .mosi_io_num = sdiPin,
        .sclk_io_num = sckPin,  
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST,&buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t spiDevCfg = {
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .mode = 0,
        .spics_io_num = csPin,  // If not using chip select, set to -1
        .queue_size = 1,
        .clock_speed_hz = 50000,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spiDevCfg, pSensorHandle));
}

void bme280_free(spi_device_handle_t sensorHandle)
{
    ESP_ERROR_CHECK(spi_bus_remove_device(sensorHandle));
}

esp_err_t bme280_spi_write(spi_device_handle_t sensorHandle, uint8_t reg, uint8_t *data, uint16_t len) {
    spi_transaction_t t;
    uint8_t tx_buffer[len + 1];
    
    tx_buffer[0] = reg & ~0x80;
    memcpy(&tx_buffer[1], data, len);
    
    memset(&t, 0, sizeof(t));
    t.length = 8 * (len + 1);
    t.tx_buffer = tx_buffer;
    
    esp_err_t ret = spi_device_polling_transmit(sensorHandle, &t);
    return ret ;
}

esp_err_t bme280_spi_read(spi_device_handle_t sensorHandle, uint8_t reg, uint8_t *data, uint16_t len) {
    spi_transaction_t t;
    uint8_t tx_buffer[len + 1];
    uint8_t rx_buffer[len + 1];
    
    tx_buffer[0] = reg | 0x80;
    memset(&tx_buffer[1], 0, len);
    
    memset(&t, 0, sizeof(t));
    t.length = 8 * (len + 1);
    t.rxlength = 8 * (len + 1);
    t.tx_buffer = tx_buffer;
    t.rx_buffer = rx_buffer;
    
    esp_err_t ret = spi_device_polling_transmit(sensorHandle, &t);
    if (ret != ESP_OK) {
        return ret;
    }
    
    memcpy(data, &rx_buffer[1], len);
    return ret;
}


void bme280_read_calibration_data(spi_device_handle_t sensorHandle, bme280_calib_data_t* calib_data){
    uint8_t reg = BME280_REG_CALIB00; // register address to read calibration data
    uint8_t buffer[26];

    ESP_ERROR_CHECK(bme280_spi_read(sensorHandle, reg, buffer, sizeof(buffer)));
    
    calib_data->dig_T1 = (uint16_t)((buffer[1] << 8) | buffer[0]);
    calib_data->dig_T2 = (int16_t)((buffer[3] << 8) | buffer[2]);
    calib_data->dig_T3 = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    calib_data->dig_P1 = (uint16_t)((buffer[7] << 8) | buffer[6]);
    calib_data->dig_P2 = (int16_t)((buffer[9] << 8) | buffer[8]);
    calib_data->dig_P3 = (int16_t)((buffer[11] << 8) | buffer[10]);
    calib_data->dig_P4 = (int16_t)((buffer[13] << 8) | buffer[12]);
    calib_data->dig_P5 = (int16_t)((buffer[15] << 8) | buffer[14]);
    calib_data->dig_P6 = (int16_t)((buffer[17] << 8) | buffer[16]);
    calib_data->dig_P7 = (int16_t)((buffer[19] << 8) | buffer[18]);
    calib_data->dig_P8 = (int16_t)((buffer[21] << 8) | buffer[20]);
    calib_data->dig_P9 = (int16_t)((buffer[23] << 8) | buffer[22]);
    
    calib_data->dig_H1 = buffer[25];

    // Read humidity calibration data
    uint8_t hum_calib_data[7];
    reg = BME280_REG_CALIB26; // register address to read humidity calibration data
    ESP_ERROR_CHECK(bme280_spi_read(sensorHandle, reg, hum_calib_data, 7));
    // ESP_ERROR_CHECK(i2c_master_transmit_receive(sensorHandle, &reg,1, buffer, 7,-1));

    calib_data->dig_H2 = (int16_t)((buffer[1] << 8) | hum_calib_data[0]);
    calib_data->dig_H3 = hum_calib_data[2];
    
    calib_data->dig_H4 = (int16_t)((hum_calib_data[3] << 4) | (hum_calib_data[4] & 0x0F));
    calib_data->dig_H5 = (int16_t)((hum_calib_data[5] << 4) | (hum_calib_data[4] >> 4));
    calib_data->dig_H6 = (int8_t)hum_calib_data[6];

}

bool bme280_configure(spi_device_handle_t sensorHandle) {
    // Configure humidity oversampling (x1)
    // Allocate buffers for transmission and reception
    // uint8_t buffer[2] = {BME280_REG_CTRL_HUM,0x01};
    uint8_t reg = BME280_REG_CTRL_HUM;
    uint8_t data = 0x01;
    
    // spi_transaction_t transCfg = {
    //     .tx_buffer = buffer,
    //     .length = sizeof(buffer) * 8,  // Length in bits 
    // };
    
    ESP_ERROR_CHECK(bme280_spi_write(sensorHandle, reg, &data, 1));
    
    // Configure temperature/pressure oversampling (x1) and normal mode
    // bits 7-5: temperature oversampling x1
    // bits 4-2: pressure oversampling x1
    // bits 1-0: normal mode
    // 0010_0111 -> 0x27
    
    // buffer[0] = BME280_REG_CTRL_MEAS;
    // buffer[1] = 0x27;
    // transCfg.tx_buffer = buffer;
    reg = BME280_REG_CTRL_MEAS;
    data = 0x27;
    ESP_ERROR_CHECK(bme280_spi_write(sensorHandle, reg, &data, 1));
    
    // Configure IIR filter and standby time
    // 0000_0000 = 0x00 - no IIR filter, standby time of 0.5ms
   
    // buffer[0] = BME280_REG_CONFIG;
    // buffer[1] = 0x00;
    // transCfg.tx_buffer = buffer;
    reg = BME280_REG_CONFIG;
    data = 0x00;
    ESP_ERROR_CHECK(bme280_spi_write(sensorHandle, reg, &data, 1));
    
    return true;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t t_fine;

int32_t bme280_compensate_temperature(int32_t raw_temp, bme280_calib_data_t* calib_data)
{
    int32_t var1, var2, T;
    var1 = ((((raw_temp>>3) - ((int32_t)calib_data->dig_T1<<1))) * ((int32_t)calib_data->dig_T2)) >> 11;
    var2 = (((((raw_temp>>4) - ((int32_t)calib_data->dig_T1)) * ((raw_temp>>4) - 
            ((int32_t)calib_data->dig_T1))) >> 12) * ((int32_t)calib_data->dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}


// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bme280_compensature_pressure(int32_t raw_press, bme280_calib_data_t* calib_data)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data->dig_P6;
    var2 = var2 + ((var1*(int64_t)calib_data->dig_P5)<<17);
    var2 = var2 + (((int64_t)calib_data->dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)calib_data->dig_P3)>>8) + ((var1 * (int64_t)calib_data->dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calib_data->dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-raw_press;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)calib_data->dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)calib_data->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data->dig_P7)<<4);
    return (uint32_t)p;
}


// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t bme280_compensate_humidity(int32_t raw_hum, bme280_calib_data_t* calib_data)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((raw_hum << 14) - (((int32_t)calib_data->dig_H4) << 20) - (((int32_t)calib_data->dig_H5) *
    v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
        ((int32_t)calib_data->dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib_data->dig_H3)) >> 11) +
        ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)calib_data->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib_data->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}




void bme280_read_data(spi_device_handle_t sensorHandle, bme280_calib_data_t* calib_data, bme280_data_t* data){
    
    uint8_t reg = 0xF7; // MSB of the data register in order to read temperature, pressure and humidity data
    uint8_t data_buffer[8]; // buffer to store the data
    // spi_transaction_t transCfg = {
    //     .tx_buffer = &reg,
    //     .rx_buffer = data_buffer,
    //     .length = 8,  // Length in bits 
    //     .rxlength = sizeof(data_buffer) * 8,
    // };
    // ESP_ERROR_CHECK(i2c_master_transmit_receive(sensorHandle, &reg, 1, data_buffer, sizeof(data_buffer), -1));
    ESP_ERROR_CHECK(bme280_spi_read(sensorHandle,reg, data_buffer, sizeof(data_buffer)));

    uint32_t raw_press = ((uint32_t)data_buffer[0] << 12) | ((uint32_t)data_buffer[1] << 4) | (data_buffer[2] >> 4);
    uint32_t raw_temp = ((uint32_t)data_buffer[3] << 12) | ((uint32_t)data_buffer[4] << 4) | (data_buffer[5] >> 4);
    uint32_t raw_hum = ((uint32_t)data_buffer[6] << 8) | data_buffer[7];

    data->temperature = bme280_compensate_temperature(raw_temp, calib_data) / 100.0f; // convert to Celsius
    data->pressure = bme280_compensature_pressure(raw_press, calib_data) / 256.0f; // convert to hPa
    data->humidity = bme280_compensate_humidity(raw_hum, calib_data) / 1024.0f; // convert to %RH


}
