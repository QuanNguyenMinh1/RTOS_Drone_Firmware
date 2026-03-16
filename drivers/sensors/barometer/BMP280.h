#ifndef __BMP280_H__
#define __BMP280_H__

#include "main.h"
#include "math.h"

/* ---- Địa chỉ I2C ---- */
#define BMP280_I2C_ADDR_LOW   0x76 << 1  // SDO = GND
#define BMP280_I2C_ADDR_HIGH  0x77 << 1  // SDO = VCC

/* ---- Thanh ghi ---- */
#define BMP280_REG_ID         0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_STATUS     0xF3
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS_MSB  0xF7
#define BMP280_REG_CALIB_START 0x88

#define BMP280_RESET_VALUE    0xB6
#define BMP280_CHIP_ID        0x58

#define SEA_PRESSURE 1013.25f

/* ---- Kiểu dữ liệu ---- */
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
} BMP280_CalibData_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;
    BMP280_CalibData_t calib;
    int32_t t_fine;
} BMP280_t;

typedef struct {
	float altitude;
	float filtered_altitude;
} BMP280_f_t;

/* ---- API ---- */
HAL_StatusTypeDef BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);
float BMP280_ReadTemperature(BMP280_t *dev);
uint32_t BMP280_ReadPressure(BMP280_t *dev);
float BMP280_ReadAltitude(BMP280_t *dev, float seaLevel_hPa);
float BMP280_ReadAltitude_2(BMP280_t *dev, float pressure, float temperature); //Get Altitude with temperature correction.
float BMP280_ReadAltitude_ToMeters(float pressure, float temperature_celsius);

#endif
