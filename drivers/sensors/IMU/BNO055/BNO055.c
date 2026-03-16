#include "BNO055.h"
#include <string.h>

/* Delay helper (blocking) */
static void bno_delay_ms(uint32_t ms) {
//    HAL_Delay(ms);

	uint32_t start = HAL_GetTick();
	while ((HAL_GetTick() - start) < ms) {
		// CPU busy-wait
	}

}

/* ---- Low-level ---- */

HAL_StatusTypeDef bno055_set_page(bno055_t *dev, uint8_t page)
{
    if (dev->current_page == page) return HAL_OK;
    HAL_StatusTypeDef st = bno055_write8(dev, BNO055_REG_PAGE_ID, page);
    if (st == HAL_OK) dev->current_page = page;
    return st;
}

HAL_StatusTypeDef bno055_write8(bno055_t *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->dev_addr, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

HAL_StatusTypeDef bno055_read8(bno055_t *dev, uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, 100);
}

HAL_StatusTypeDef bno055_readlen(bno055_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->dev_addr, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/* ---- Init & Modes ---- */

HAL_StatusTypeDef bno055_init(bno055_t *dev, I2C_HandleTypeDef *hi2c, uint16_t dev_addr)
{
    dev->hi2c = hi2c;
    dev->dev_addr = dev_addr;
    dev->current_page = 0xFF; // force write on first set

    /* Về PAGE 0 */
    if (bno055_set_page(dev, BNO055_PAGE0) != HAL_OK) return HAL_ERROR;

    /* Kiểm tra CHIP ID */
    uint8_t id = 0;
    if (bno055_read8(dev, BNO055_REG_CHIP_ID, &id) != HAL_OK) return HAL_ERROR;
    if (id != BNO055_CHIP_ID_VALUE) {
        /* Datasheet gợi ý chờ sau reset nguồn */
        bno_delay_ms(50);
        if (bno055_read8(dev, BNO055_REG_CHIP_ID, &id) != HAL_OK) return HAL_ERROR;
        if (id != BNO055_CHIP_ID_VALUE) return HAL_ERROR;
    }

    /* Vào CONFIG mode trước khi đổi thiết lập */
    if (bno055_set_opr_mode(dev, BNO055_OPR_MODE_CONFIG) != HAL_OK) return HAL_ERROR;

    /* Cấu hình đơn vị (m/s^2, dps, deg, °C, Android) */
    if (bno055_config_units_default(dev) != HAL_OK) return HAL_ERROR;

    /* Power mode = Normal */
    if (bno055_set_power_mode(dev, BNO055_POWER_NORMAL) != HAL_OK) return HAL_ERROR;

    /* Trở về PAGE 0 cho data */
    if (bno055_set_page(dev, BNO055_PAGE0) != HAL_OK) return HAL_ERROR;

    /* Chuyển sang NDOF (fusion) – tuỳ nhu cầu thay bằng IMU/COMPASS... */
    if (bno055_set_opr_mode(dev, BNO055_OPR_MODE_NDOF) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef bno055_set_opr_mode(bno055_t *dev, bno055_opr_mode_t mode)
{
    /* Chuyển MODE cần khoảng trễ theo datasheet */
    if (bno055_set_page(dev, BNO055_PAGE0) != HAL_OK) return HAL_ERROR;
    if (bno055_write8(dev, BNO055_REG_OPR_MODE, (uint8_t)mode) != HAL_OK) return HAL_ERROR;

    if (mode == BNO055_OPR_MODE_CONFIG) {
        bno_delay_ms(20);
    } else {
        bno_delay_ms(10);
    }
    return HAL_OK;
}

HAL_StatusTypeDef bno055_set_power_mode(bno055_t *dev, bno055_power_mode_t mode)
{
    if (bno055_set_page(dev, BNO055_PAGE0) != HAL_OK) return HAL_ERROR;
    return bno055_write8(dev, BNO055_REG_PWR_MODE, (uint8_t)mode);
}

HAL_StatusTypeDef bno055_reset(bno055_t *dev)
{
    if (bno055_set_opr_mode(dev, BNO055_OPR_MODE_CONFIG) != HAL_OK) return HAL_ERROR;
    if (bno055_write8(dev, BNO055_REG_SYS_TRIGGER, BNO055_SYS_TRIGGER_RST_SYS) != HAL_OK) return HAL_ERROR;
    /* Datasheet: chờ 650ms+ sau reset */
    bno_delay_ms(700);
    dev->current_page = 0xFF;
    /* Thường cần init lại */
    return HAL_OK;
}

HAL_StatusTypeDef bno055_config_units_default(bno055_t *dev)
{
    if (bno055_set_page(dev, BNO055_PAGE0) != HAL_OK) return HAL_ERROR;

    uint8_t unit =
        (BNO055_UNIT_SEL_ACC_MS2) |
        (BNO055_UNIT_SEL_ANG_RATE_DPS) |
        (BNO055_UNIT_SEL_EUL_DEG) |
        (BNO055_UNIT_SEL_TEMP_C) |
        (BNO055_UNIT_SEL_ORI_ANDROID);

    return bno055_write8(dev, BNO055_REG_UNIT_SEL, unit);
}

/* ---- Helpers đọc vector/quaternion ---- */

static HAL_StatusTypeDef read_vec3_i16(bno055_t *dev, uint8_t base_reg, bno055_vec3i16_t *out)
{
    uint8_t buf[6];
    if (bno055_readlen(dev, base_reg, buf, 6) != HAL_OK) return HAL_ERROR;
    out->x = (int16_t)((buf[1] << 8) | buf[0]);
    out->y = (int16_t)((buf[3] << 8) | buf[2]);
    out->z = (int16_t)((buf[5] << 8) | buf[4]);
    return HAL_OK;
}

HAL_StatusTypeDef bno055_read_acc_i16(bno055_t *dev, bno055_vec3i16_t *v) {
    return read_vec3_i16(dev, BNO055_REG_ACC_DATA_X_LSB, v);
}
HAL_StatusTypeDef bno055_read_mag_i16(bno055_t *dev, bno055_vec3i16_t *v) {
    return read_vec3_i16(dev, BNO055_REG_MAG_DATA_X_LSB, v);
}
HAL_StatusTypeDef bno055_read_gyr_i16(bno055_t *dev, bno055_vec3i16_t *v) {
    return read_vec3_i16(dev, BNO055_REG_GYR_DATA_X_LSB, v);
}
HAL_StatusTypeDef bno055_read_lia_i16(bno055_t *dev, bno055_vec3i16_t *v) {
    return read_vec3_i16(dev, BNO055_REG_LIA_DATA_X_LSB, v);
}
HAL_StatusTypeDef bno055_read_grv_i16(bno055_t *dev, bno055_vec3i16_t *v) {
    return read_vec3_i16(dev, BNO055_REG_GRV_DATA_X_LSB, v);
}

HAL_StatusTypeDef bno055_read_euler_i16(bno055_t *dev, bno055_euler_i16_t *e)
{
    uint8_t buf[6];
    if (bno055_readlen(dev, BNO055_REG_EUL_HEADING_LSB, buf, 6) != HAL_OK) return HAL_ERROR;
    e->heading = (int16_t)((buf[1] << 8) | buf[0]);
    e->roll    = (int16_t)((buf[3] << 8) | buf[2]);
    e->pitch   = (int16_t)((buf[5] << 8) | buf[4]);
    return HAL_OK;
}

HAL_StatusTypeDef bno055_read_quat_i16(bno055_t *dev, bno055_quat_i16_t *q)
{
    uint8_t buf[8];
    if (bno055_readlen(dev, BNO055_REG_QUA_DATA_W_LSB, buf, 8) != HAL_OK) return HAL_ERROR;
    q->w = (int16_t)((buf[1] << 8) | buf[0]);
    q->x = (int16_t)((buf[3] << 8) | buf[2]);
    q->y = (int16_t)((buf[5] << 8) | buf[4]);
    q->z = (int16_t)((buf[7] << 8) | buf[6]);
    return HAL_OK;
}

HAL_StatusTypeDef bno055_read_temp_i8(bno055_t *dev, int8_t *t_c)
{
    uint8_t u = 0;
    if (bno055_read8(dev, BNO055_REG_TEMP, &u) != HAL_OK) return HAL_ERROR;
    *t_c = (int8_t)u; // theo UNIT_SEL hiện tại; mặc định là °C
    return HAL_OK;
}

HAL_StatusTypeDef bno055_get_calibration_status(bno055_t *dev, bno055_calib_status_t *out)
{
    uint8_t val;
    if (bno055_read8(dev, BNO055_REG_CALIB_STAT, &val) != HAL_OK)
        return HAL_ERROR;

    out->mag   =  val        & 0x03;
    out->accel = (val >> 2)  & 0x03;
    out->gyro  = (val >> 4)  & 0x03;
    out->sys   = (val >> 6)  & 0x03;

    return HAL_OK;
}

HAL_StatusTypeDef bno055_set_calibration_profile(bno055_t *dev, const uint8_t *calib_data) {
    HAL_StatusTypeDef status;

    // -----------------------------------------------------------
    // BƯỚC 1: Select the operation mode to CONFIG_MODE
    // (Chuyển sang chế độ CONFIG để cho phép ghi dữ liệu hiệu chuẩn)
    // -----------------------------------------------------------
    status = bno055_set_opr_mode(dev, BNO055_OPR_MODE_CONFIG);
    if (status != HAL_OK) {
        return status;
    }
    // Đợi 19ms theo Datasheet (tối đa) để chuyển chế độ
    HAL_Delay(20);

    // -----------------------------------------------------------
    // BƯỚC 2: Write the corresponding sensor offsets and radius data
    // (Ghi 22 byte dữ liệu Calibration Profile)
    // -----------------------------------------------------------
    // Đảm bảo đang ở Page 0 trước khi ghi thanh ghi từ 0x55
    status = bno055_set_page(dev, BNO055_PAGE0);
    if (status != HAL_OK) {
        return status;
    }

    // Ghi 22 byte từ 0x55 (MAG_OFFSET_X_LSB)
    // Giả định hàm bno055_write_len (hoặc bno055_readlen) có thể dùng để ghi một khối
    // Bạn có thể cần triển khai hàm ghi khối nếu chưa có.
    // Nếu không, bạn cần ghi từng byte.

    // Sử dụng hàm HAL_I2C_Mem_Write nếu cần (không có trong driver mẫu)
    // Giả sử có hàm ghi khối tương đương:
    status = HAL_I2C_Mem_Write(
        dev->hi2c,
        dev->dev_addr,
        BNO055_REG_OFFSET_START, // Địa chỉ thanh ghi bắt đầu: 0x55
        I2C_MEMADD_SIZE_8BIT,    // Địa chỉ thanh ghi là 8-bit
        (uint8_t *)calib_data,   // Dữ liệu cần ghi
        22,                      // Độ dài 22 byte
        HAL_MAX_DELAY
    );
    if (status != HAL_OK) {
        return status;
    }
    // Chờ BNO055 xử lý ghi vào bộ nhớ
    HAL_Delay(10);


    // -----------------------------------------------------------
    // BƯỚC 3: Change operation mode to fusion mode
    // (Chuyển sang chế độ hoạt động bình thường, ví dụ: NDOF)
    // -----------------------------------------------------------
    status = bno055_set_opr_mode(dev, BNO055_OPR_MODE_NDOF); // Thay NDOF bằng chế độ fusion bạn muốn
    if (status != HAL_OK) {
        return status;
    }
    // Đợi 7ms theo Datasheet
    HAL_Delay(10);

    return HAL_OK;
}
/* ---- Convert sang float ---- */

void bno055_euler_i16_to_f(const bno055_euler_i16_t *in, bno055_euler_f_t *out)
{
    /* Euler LSB = 1/16 deg nếu UNIT_SEL chọn DEG */
    const float scale = 1.0f / 16.0f;
    out->heading = (float)in->heading * scale;
    out->roll    = (float)in->roll    * scale;
    out->pitch   = (float)in->pitch   * scale;
}

void bno055_quat_i16_to_f(const bno055_quat_i16_t *in, bno055_quat_f_t *out)
{
    /* Quaternion LSB = 1/16384 (datasheet) */
    const float scale = 1.0f / 16384.0f;
    out->w = (float)in->w * scale;
    out->x = (float)in->x * scale;
    out->y = (float)in->y * scale;
    out->z = (float)in->z * scale;
}

void bno055_read_gyr_i16_to_f(const bno055_vec3i16_t *in, bno055_vec3f_t *out)
{
    // Gyro LSB = 16 LSB/deg/s (vì bạn đã set BNO055_UNIT_SEL_ANG_RATE_DPS)
    const float scale = 1.0f / 16.0f;	// chia Gyro LSB
    out->x = (float)in->x * scale;
    out->y = (float)in->y * scale;
    out->z = (float)in->z * scale;
}

void bno055_read_acc_i16_to_f(const bno055_vec3i16_t *in, bno055_vec3f_t *out)
{
    // LSB = 1/100 m/s^2
    const float scale = 1.0f / 100.0f;
    out->x = (float)in->x * scale;
    out->y = (float)in->y * scale;
    out->z = (float)in->z * scale;
}

HAL_StatusTypeDef bno055_set_axis_remap(bno055_t *dev, uint8_t axis_map, uint8_t axis_sign)
{
    // Phải ở CONFIG mode
    if (bno055_set_opr_mode(dev, BNO055_OPR_MODE_CONFIG) != HAL_OK)
        return HAL_ERROR;

    if (bno055_set_page(dev, BNO055_PAGE0) != HAL_OK)
        return HAL_ERROR;

    // Ghi AXIS_MAP_CONFIG (0x41)
    if (bno055_write8(dev, BNO055_REG_AXIS_MAP_CONFIG, axis_map) != HAL_OK)
        return HAL_ERROR;

    // Ghi AXIS_MAP_SIGN (0x42) nếu cần đảo dấu
    if (bno055_write8(dev, BNO055_REG_AXIS_MAP_SIGN, axis_sign) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(10);

    // Trở về operating mode
    if (bno055_set_opr_mode(dev, BNO055_OPR_MODE_NDOF) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}
