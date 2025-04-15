#ifndef __BME280_SENSOR_I2C_H__INCLUDED__
#define __BME280_SENSOR_I2C_H__INCLUDED__

#include "driver/i2c_master.h"

// #define TC74_A1_SENSOR_ADDR	        0x49
// #define TC74_A5_SENSOR_ADDR         0x4D
// #define TC74_SCL_DFLT_FREQ_HZ       50000

#define BME280_SLAVE_I2C_ADDR 0x77
#define BME280_SCL_DFLT_FREQ_HZ 50000

// BME280 registers
#define BME280_REG_CALIB00      0x88
#define BME280_REG_CALIB26      0xE1
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5


// BME280 calibration data structure
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_data_t;


typedef struct {
    float temperature;
    float pressure;
    float humidity;
} bme280_data_t;

void bme280_init(i2c_master_bus_handle_t* pBusHandle,
               i2c_master_dev_handle_t* pSensorHandle,
               uint8_t sensorAddr, int sdaPin, int sclPin, uint32_t clkSpeedHz);

void bme280_free(i2c_master_bus_handle_t busHandle,
               i2c_master_dev_handle_t sensorHandle);



void bme280_read_calibration_data(i2c_master_dev_handle_t sensorHandle, bme280_calib_data_t* calib_data);

bool bme280_configure(i2c_master_dev_handle_t sensorHandle);

void bme280_read_data(i2c_master_dev_handle_t sensorHandle, bme280_calib_data_t* calib_data, 
                    bme280_data_t* data);



#endif // __BME280_SENSOR_I2C_H__INCLUDED__
