#include "bme280_sensor_i2c.h"

void bme280_init(i2c_master_bus_handle_t* pBusHandle,
                i2c_master_dev_handle_t* pSensorHandle,
                uint8_t sensorAddr, int sdaPin, int sclPin, uint32_t clkSpeedHz)
{
    i2c_master_bus_config_t i2cMasterCfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = sclPin,
        .sda_io_num = sdaPin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2cMasterCfg, pBusHandle));

    i2c_device_config_t i2cDevCfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = sensorAddr,
        .scl_speed_hz = clkSpeedHz,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(*pBusHandle, &i2cDevCfg, pSensorHandle));
}

void bme280_free(i2c_master_bus_handle_t busHandle,
                i2c_master_dev_handle_t sensorHandle)
{
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(sensorHandle));
    ESP_ERROR_CHECK(i2c_del_master_bus(busHandle));
}

void bme280_read_calibration_data(i2c_master_dev_handle_t sensorHandle, bme280_calib_data_t* calib_data){
    uint8_t buffer[26];
    uint8_t reg = BME280_REG_CALIB00; // register address to read calibration data
    ESP_ERROR_CHECK(i2c_master_transmit_receive(sensorHandle, &reg, 1, buffer, 26, -1));
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
    reg = BME280_REG_CALIB26; // register address to read humidity calibration data
    ESP_ERROR_CHECK(i2c_master_transmit_receive(sensorHandle, &reg,1, buffer, 7,-1));

    calib_data->dig_H2 = (int16_t)((buffer[1] << 8) | buffer[0]);
    calib_data->dig_H3 = buffer[2];
    
    calib_data->dig_H4 = (int16_t)((buffer[3] << 4) | (buffer[4] & 0x0F));
    calib_data->dig_H5 = (int16_t)((buffer[5] << 4) | (buffer[4] >> 4));
    calib_data->dig_H6 = (int8_t)buffer[6];

}

bool bme280_configure(i2c_master_dev_handle_t sensorHandle) {
    // Configure humidity oversampling (x1)
    uint8_t buffer[2] = {BME280_REG_CTRL_HUM,0x01};
    ESP_ERROR_CHECK(i2c_master_transmit(sensorHandle,buffer,sizeof(buffer),-1));
    
    
    // Configure temperature/pressure oversampling (x1) and normal mode
    // bits 7-5: temperature oversampling x1
    // bits 4-2: pressure oversampling x1
    // bits 1-0: normal mode
    // 0010_0111 -> 0x27
    buffer[0] = BME280_REG_CTRL_MEAS;
    buffer[1] = 0x27;
    ESP_ERROR_CHECK(i2c_master_transmit(sensorHandle, buffer, sizeof(buffer), -1));
    
    // Configure IIR filter and standby time
    // 0000_0000 = 0x00 - no IIR filter, standby time of 0.5ms
    buffer[0] = BME280_REG_CONFIG;
    buffer[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit(sensorHandle, buffer, sizeof(buffer), -1));
    
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




void bme280_read_data(i2c_master_dev_handle_t sensorHandle, bme280_calib_data_t* calib_data, bme280_data_t* data){
    
    uint8_t reg = 0xF7; // MSB of the data register in order to read temperature, pressure and humidity data
    uint8_t data_buffer[8]; // buffer to store the data
    
    ESP_ERROR_CHECK(i2c_master_transmit_receive(sensorHandle, &reg, 1, data_buffer, sizeof(data_buffer), -1));

    uint32_t raw_press = ((uint32_t)data_buffer[0] << 12) | ((uint32_t)data_buffer[1] << 4) | (data_buffer[2] >> 4);
    uint32_t raw_temp = ((uint32_t)data_buffer[3] << 12) | ((uint32_t)data_buffer[4] << 4) | (data_buffer[5] >> 4);
    uint32_t raw_hum = ((uint32_t)data_buffer[6] << 8) | data_buffer[7];

    data->temperature = bme280_compensate_temperature(raw_temp, calib_data) / 100.0f; // convert to Celsius
    data->pressure = bme280_compensature_pressure(raw_press, calib_data) / 256.0f; // convert to hPa
    data->humidity = bme280_compensate_humidity(raw_hum, calib_data) / 1024.0f; // convert to %RH


}
