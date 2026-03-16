/*
 * BNO055 STM32 HAL C Driver (minimal but practical)
 * MIT License – adapted from DFRobot_BNO055 (Arduino/C++)
 *
 * I2C note: For STM32 HAL, device address must be left-shifted by 1.
 *           Default addresses: 0x28 (COM3 LOW) or 0x29 (COM3 HIGH).
 */

#ifndef BNO055_STM32_HAL_H
#define BNO055_STM32_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t dev_addr;   // ví dụ: (0x28 << 1) hoặc (0x29 << 1)
    uint8_t  current_page;
} bno055_t;

/* --- Các thanh ghi chính (Page 0) --- */
#define BNO055_REG_CHIP_ID        0x00
#define BNO055_REG_PAGE_ID        0x07
#define BNO055_REG_ACC_DATA_X_LSB 0x08
#define BNO055_REG_MAG_DATA_X_LSB 0x0E
#define BNO055_REG_GYR_DATA_X_LSB 0x14
#define BNO055_REG_EUL_HEADING_LSB 0x1A
#define BNO055_REG_QUA_DATA_W_LSB  0x20
#define BNO055_REG_LIA_DATA_X_LSB  0x28
#define BNO055_REG_GRV_DATA_X_LSB  0x2E
#define BNO055_REG_TEMP            0x34
#define BNO055_REG_UNIT_SEL        0x3B
#define BNO055_REG_OPR_MODE        0x3D
#define BNO055_REG_PWR_MODE        0x3E
#define BNO055_REG_SYS_TRIGGER     0x3F
#define BNO055_REG_AXIS_MAP_CONFIG 0x41
#define BNO055_REG_AXIS_MAP_SIGN   0x42

#define BNO055_REG_CALIB_STAT	   0x35
#define BNO055_REG_OFFSET_START     0x55 // MAG_OFFSET_X_LSB
/* --- Các thanh ghi Page 1 (cấu hình chi tiết cảm biến) --- */
#define BNO055_PAGE1               0x01
#define BNO055_PAGE0               0x00
#define BNO055_REG_ACC_CONFIG      0x08  // Page 1
#define BNO055_REG_MAG_CONFIG      0x09  // Page 1
#define BNO055_REG_GYR_CONFIG0     0x0A  // Page 1
#define BNO055_REG_GYR_CONFIG1     0x0B  // Page 1

/* --- ID mặc định --- */
#define BNO055_CHIP_ID_VALUE       0xA0

/* --- Power mode --- */
typedef enum {
    BNO055_POWER_NORMAL   = 0x00,
    BNO055_POWER_LOWPOWER = 0x01,
    BNO055_POWER_SUSPEND  = 0x02
} bno055_power_mode_t;

/* --- Operation mode --- (datasheet order) */
typedef enum {
    BNO055_OPR_MODE_CONFIG     = 0x00,
    BNO055_OPR_MODE_ACCONLY    = 0x01,
    BNO055_OPR_MODE_MAGONLY    = 0x02,
    BNO055_OPR_MODE_GYRONLY    = 0x03,
    BNO055_OPR_MODE_ACCMAG     = 0x04,
    BNO055_OPR_MODE_ACCGYR     = 0x05,
    BNO055_OPR_MODE_MAGGYR     = 0x06,
    BNO055_OPR_MODE_AMG        = 0x07,
    BNO055_OPR_MODE_IMU        = 0x08,
    BNO055_OPR_MODE_COMPASS    = 0x09,
    BNO055_OPR_MODE_M4G        = 0x0A,
    BNO055_OPR_MODE_NDOF_FMC   = 0x0B,
    BNO055_OPR_MODE_NDOF       = 0x0C
} bno055_opr_mode_t;

/* --- Unit selection bits (UNIT_SEL, 0x3B) ---
   Bạn có thể chỉnh cho phù hợp nhu cầu. Dưới đây là cấu hình mặc định phổ biến:
   ACC = m/s^2, GYR = dps, EUL = deg, TEMP = °C, Orientation = Android.
*/
#define BNO055_UNIT_SEL_ACC_MS2    (0<<0)  // 0: m/s^2, 1: mg
#define BNO055_UNIT_SEL_ANG_RATE_DPS (0<<1) // 0: dps, 1: rps
#define BNO055_UNIT_SEL_EUL_DEG    (0<<2)  // 0: deg, 1: rad
#define BNO055_UNIT_SEL_TEMP_C     (0<<4)  // 0: °C,  1: °F
#define BNO055_UNIT_SEL_ORI_ANDROID (0<<7) // 0: Android, 1: Windows

/* --- System trigger bits --- */
#define BNO055_SYS_TRIGGER_RST_SYS (1<<5)

/* --- Cấu trúc dữ liệu --- */
typedef struct {
    int16_t x, y, z;
} bno055_vec3i16_t;

typedef struct {
    float x, y, z;
} bno055_vec3f_t;

typedef struct {
    int16_t w, x, y, z;
} bno055_quat_i16_t;

typedef struct {
    float w, x, y, z;
} bno055_quat_f_t;

typedef struct {
    int16_t heading, roll, pitch;   // raw (LSB), chưa convert sang float
} bno055_euler_i16_t;

typedef struct {
    float heading, roll, pitch;     // float (deg nếu chọn DEG)
} bno055_euler_f_t;


typedef struct {
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} bno055_calib_status_t;

/* --- Hàm API chính --- */
HAL_StatusTypeDef bno055_init(bno055_t *dev, I2C_HandleTypeDef *hi2c, uint16_t dev_addr);

HAL_StatusTypeDef bno055_set_page(bno055_t *dev, uint8_t page);
HAL_StatusTypeDef bno055_write8(bno055_t *dev, uint8_t reg, uint8_t val);
HAL_StatusTypeDef bno055_read8 (bno055_t *dev, uint8_t reg, uint8_t *val);
HAL_StatusTypeDef bno055_readlen(bno055_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);

HAL_StatusTypeDef bno055_set_opr_mode(bno055_t *dev, bno055_opr_mode_t mode);
HAL_StatusTypeDef bno055_set_power_mode(bno055_t *dev, bno055_power_mode_t mode);
HAL_StatusTypeDef bno055_reset(bno055_t *dev);

/* Unit = deg/dps/ms2/°C theo cấu hình ở UNIT_SEL mặc định ở trên */
HAL_StatusTypeDef bno055_config_units_default(bno055_t *dev);

/* Đọc raw */
HAL_StatusTypeDef bno055_read_acc_i16(bno055_t *dev, bno055_vec3i16_t *v);
HAL_StatusTypeDef bno055_read_mag_i16(bno055_t *dev, bno055_vec3i16_t *v);
HAL_StatusTypeDef bno055_read_gyr_i16(bno055_t *dev, bno055_vec3i16_t *v);
HAL_StatusTypeDef bno055_read_lia_i16(bno055_t *dev, bno055_vec3i16_t *v);
HAL_StatusTypeDef bno055_read_grv_i16(bno055_t *dev, bno055_vec3i16_t *v);
HAL_StatusTypeDef bno055_read_euler_i16(bno055_t *dev, bno055_euler_i16_t *e);
HAL_StatusTypeDef bno055_read_quat_i16(bno055_t *dev, bno055_quat_i16_t *q);
HAL_StatusTypeDef bno055_read_temp_i8(bno055_t *dev, int8_t *t_c);

HAL_StatusTypeDef bno055_set_axis_remap(bno055_t *dev, uint8_t axis_map, uint8_t axis_sign);


/* Đọc float (dùng hệ số scale thường gặp trong datasheet) */
void bno055_euler_i16_to_f(const bno055_euler_i16_t *in, bno055_euler_f_t *out);
void bno055_quat_i16_to_f (const bno055_quat_i16_t  *in, bno055_quat_f_t  *out);
void bno055_read_acc_i16_to_f(const bno055_vec3i16_t *in, bno055_vec3f_t *out);
void bno055_read_gyr_i16_to_f(const bno055_vec3i16_t *in, bno055_vec3f_t *out);

/* Lưu ý:
   - Euler LSB = 1/16 deg
   - Gyro LSB = 1/16 dps
   - Mag LSB  = 1/16 uT  (nếu chọn đơn vị uT; ở đây minh hoạ đọc raw)
   - Acc/LinAcc/Gravity: tuỳ UNIT_SEL; bạn có thể tự scale theo m/s^2 hoặc mg.
*/

#ifdef __cplusplus
}
#endif
#endif /* BNO055_STM32_HAL_H */
