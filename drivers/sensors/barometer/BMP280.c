#include "bmp280.h"

/* ---- Hàm đọc ghi cơ bản ---- */
static HAL_StatusTypeDef bmp280_read(BMP280_t *dev, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(dev->hi2c, dev->addr, reg, 1, data, len, 100);
}

static HAL_StatusTypeDef bmp280_write(BMP280_t *dev, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(dev->hi2c, dev->addr, reg, 1, &val, 1, 100);
}

/* ---- Đọc hệ số hiệu chỉnh ---- */
static void bmp280_read_calib_data(BMP280_t *dev) {
    uint8_t calib[24];
    bmp280_read(dev, BMP280_REG_CALIB_START, calib, 24);

    dev->calib.dig_T1 = (uint16_t)((calib[1] << 8) | calib[0]);
    dev->calib.dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
    dev->calib.dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);
    dev->calib.dig_P1 = (uint16_t)((calib[7] << 8) | calib[6]);
    dev->calib.dig_P2 = (int16_t)((calib[9] << 8) | calib[8]);
    dev->calib.dig_P3 = (int16_t)((calib[11] << 8) | calib[10]);
    dev->calib.dig_P4 = (int16_t)((calib[13] << 8) | calib[12]);
    dev->calib.dig_P5 = (int16_t)((calib[15] << 8) | calib[14]);
    dev->calib.dig_P6 = (int16_t)((calib[17] << 8) | calib[16]);
    dev->calib.dig_P7 = (int16_t)((calib[19] << 8) | calib[18]);
    dev->calib.dig_P8 = (int16_t)((calib[21] << 8) | calib[20]);
    dev->calib.dig_P9 = (int16_t)((calib[23] << 8) | calib[22]);
}

/* ---- Khởi tạo ---- */
HAL_StatusTypeDef BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr) {
    dev->hi2c = hi2c;
    dev->addr = addr;

    uint8_t id = 0;
    if (bmp280_read(dev, BMP280_REG_ID, &id, 1) != HAL_OK || id != BMP280_CHIP_ID)
        return HAL_ERROR;

    bmp280_write(dev, BMP280_REG_RESET, BMP280_RESET_VALUE);
    HAL_Delay(100);

    bmp280_read_calib_data(dev);

    // Cấu hình oversampling ×1, mode normal
    uint8_t ctrl_meas = (1 << 5) | (1 << 2) | 0x03;
    uint8_t config = (4 << 5); // standby 125ms
    bmp280_write(dev, BMP280_REG_CTRL_MEAS, ctrl_meas);
    bmp280_write(dev, BMP280_REG_CONFIG, config);

    return HAL_OK;
}

/* ---- Đọc nhiệt độ ---- */
float BMP280_ReadTemperature(BMP280_t *dev) {
    uint8_t buf[3];
    bmp280_read(dev, 0xFA, buf, 3);
    int32_t adc_T = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);

    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) * ((int32_t)dev->calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->calib.dig_T1))) >> 12) * ((int32_t)dev->calib.dig_T3)) >> 14;
    dev->t_fine = var1 + var2;

    float T = (dev->t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

/* ---- Đọc áp suất ---- */
uint32_t BMP280_ReadPressure(BMP280_t *dev) {
    uint8_t buf[3];
    bmp280_read(dev, 0xF7, buf, 3);
    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);

    int64_t var1, var2, p;
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) + ((var1 * (int64_t)dev->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calib.dig_P1) >> 33;

    if (var1 == 0)
        return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);

    return (uint32_t)(p / 256);
}

/* ---- Tính độ cao ---- */
float BMP280_ReadAltitude(BMP280_t *dev, float seaLevel_hPa) {
    float pressure = BMP280_ReadPressure(dev) / 100.0f;
    float temp = 44330.0f * (1.0f - powf(pressure / seaLevel_hPa, 0.1903f));
    if (temp >= 0)
    {
        return temp;

    }
    else
    {

    }
}

float BMP280_ReadAltitude_2(BMP280_t *dev, float pressure, float temperature) //Get Altitude with temperature correction.
{
	return ((powf((SEA_PRESSURE / pressure), 0.1902226f) - 1.0f) * (temperature + 273.15f)) / 0.0065f;
}

float BMP280_ReadAltitude_ToMeters(float pressure, float temperature_celsius)
{
    // Đảm bảo SEA_PRESSURE và pressure có cùng đơn vị.
    // Nếu dùng hPa/mbar: SEA_PRESSURE = 1013.25f
    // Nếu dùng Pa: SEA_PRESSURE = 101325.0f (nên dùng Pa nếu áp suất đầu ra của BMP280 là Pa)

    // Giả sử bạn đang dùng đơn vị hPa/mbar (1013.25f)
    const float SEA_PRESSURE_HPA = 1013.25f;
    const float TEMP_LAPSE_RATE = 0.0065f; // L (K/m)
    const float EXPONENT = 0.1902226f;     // R*L / (g*M)
    float pressure_hpa = pressure / 100.0f; // Chuyển từ Pa sang hPa

    // Công thức tính độ cao ra mét:
    float altitude = ((temperature_celsius + 273.15f) / TEMP_LAPSE_RATE) * (1.0f - powf((pressure_hpa / SEA_PRESSURE_HPA), EXPONENT));
    return altitude; // Kết quả là mét
}
