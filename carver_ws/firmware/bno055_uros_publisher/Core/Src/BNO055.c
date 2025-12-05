/*
 * BNO055.c
 *
 *  Created on: Aug 6, 2025
 *      Author: transporter
 */

#include "BNO055.h"

static HAL_StatusTypeDef WriteReg(BNO055_t *bno, uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(bno->i2c, bno->address, reg, 1, &value, 1, 30);
    if (status != HAL_OK) {
        bno->error_count++;
    }
    return status;
}

static HAL_StatusTypeDef ReadReg(BNO055_t *bno, uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(bno->i2c, bno->address, reg, 1, value, 1, 30);
    if (status != HAL_OK) {
        bno->error_count++;
    }
    return status;
}

static HAL_StatusTypeDef ReadRegs(BNO055_t *bno, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(bno->i2c, bno->address, reg, 1, buffer, len, 30);
    if (status != HAL_OK) {
        bno->error_count++;
    }
    return status;
}

HAL_StatusTypeDef BNO055_Init(BNO055_t *bno, I2C_HandleTypeDef *i2c, uint8_t addr)
{
    uint8_t chip_id = 0;

    memset(bno, 0, sizeof(BNO055_t));
    bno->i2c = i2c;
    bno->address = (addr == 0) ? BNO055_ADDRESS_A : BNO055_ADDRESS_B;
    bno->dma_ready = true;
    bno->current_mode = BNO055_MODE_CONFIG;

    bno->current_remap_config = AXIS_REMAP_P1;
    bno->current_remap_sign = AXIS_REMAP_SIGN_P1;

    HAL_Delay(50);

    for (int i = 0; i < 5; i++) {
        if (ReadReg(bno, BNO055_CHIP_ID_ADDR, &chip_id) == HAL_OK && chip_id == BNO055_ID)
            break;
        HAL_Delay(10);
    }

    if (chip_id != BNO055_ID) {
        if (WriteReg(bno, BNO055_SYS_TRIGGER_ADDR, 0x20) != HAL_OK)
            return HAL_ERROR;
        HAL_Delay(650);
        if (ReadReg(bno, BNO055_CHIP_ID_ADDR, &chip_id) != HAL_OK || chip_id != BNO055_ID)
            return HAL_ERROR;
    }

    if (WriteReg(bno, BNO055_OPR_MODE_ADDR, BNO055_MODE_CONFIG) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(20);

    WriteReg(bno, BNO055_PWR_MODE_ADDR, 0x00);
    WriteReg(bno, BNO055_PAGE_ID_ADDR, 0x00);
    WriteReg(bno, BNO055_UNIT_SEL_ADDR, 0x0C);
    WriteReg(bno, BNO055_SYS_TRIGGER_ADDR, 0x00);

    if (BNO055_SetMode(bno, BNO055_MODE_NDOF) != HAL_OK)
        return HAL_ERROR;

    bno->last_update = HAL_GetTick();
    bno->error_count = 0;

    return HAL_OK;
}

HAL_StatusTypeDef BNO055_SetMode(BNO055_t *bno, uint8_t mode)
{
    uint8_t current_mode;

    if (ReadReg(bno, BNO055_OPR_MODE_ADDR, &current_mode) != HAL_OK)
        return HAL_ERROR;

    if (current_mode != BNO055_MODE_CONFIG) {
        if (WriteReg(bno, BNO055_OPR_MODE_ADDR, BNO055_MODE_CONFIG) != HAL_OK)
            return HAL_ERROR;
        HAL_Delay(20);
    }

    if (WriteReg(bno, BNO055_OPR_MODE_ADDR, mode) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(20);

    if (ReadReg(bno, BNO055_OPR_MODE_ADDR, &current_mode) != HAL_OK)
        return HAL_ERROR;

    if (current_mode != mode)
        return HAL_ERROR;

    bno->current_mode = mode;
    return HAL_OK;
}

HAL_StatusTypeDef BNO055_SetAxisRemap(BNO055_t *bno, axis_remap_config_t config, axis_remap_sign_t sign)
{
    uint8_t saved_mode = bno->current_mode;

    if (BNO055_SetMode(bno, BNO055_MODE_CONFIG) != HAL_OK)
        return HAL_ERROR;

    if (WriteReg(bno, BNO055_AXIS_MAP_CONFIG_ADDR, config) != HAL_OK)
        return HAL_ERROR;
    if (WriteReg(bno, BNO055_AXIS_MAP_SIGN_ADDR, sign) != HAL_OK)
        return HAL_ERROR;

    bno->current_remap_config = config;
    bno->current_remap_sign = sign;

    return BNO055_SetMode(bno, saved_mode);
}

HAL_StatusTypeDef BNO055_LoadCalibration(BNO055_t *bno, const calibration_data_t *calib)
{
    uint8_t buffer[22];
    uint8_t saved_mode = bno->current_mode;

    if (calib->accel_radius == 0 || calib->mag_radius == 0)
        return HAL_ERROR;

    if (BNO055_SetMode(bno, BNO055_MODE_CONFIG) != HAL_OK)
        return HAL_ERROR;

    buffer[0] = calib->accel_offset_x & 0xFF;
    buffer[1] = (calib->accel_offset_x >> 8) & 0xFF;
    buffer[2] = calib->accel_offset_y & 0xFF;
    buffer[3] = (calib->accel_offset_y >> 8) & 0xFF;
    buffer[4] = calib->accel_offset_z & 0xFF;
    buffer[5] = (calib->accel_offset_z >> 8) & 0xFF;
    buffer[6] = calib->mag_offset_x & 0xFF;
    buffer[7] = (calib->mag_offset_x >> 8) & 0xFF;
    buffer[8] = calib->mag_offset_y & 0xFF;
    buffer[9] = (calib->mag_offset_y >> 8) & 0xFF;
    buffer[10] = calib->mag_offset_z & 0xFF;
    buffer[11] = (calib->mag_offset_z >> 8) & 0xFF;
    buffer[12] = calib->gyro_offset_x & 0xFF;
    buffer[13] = (calib->gyro_offset_x >> 8) & 0xFF;
    buffer[14] = calib->gyro_offset_y & 0xFF;
    buffer[15] = (calib->gyro_offset_y >> 8) & 0xFF;
    buffer[16] = calib->gyro_offset_z & 0xFF;
    buffer[17] = (calib->gyro_offset_z >> 8) & 0xFF;
    buffer[18] = calib->accel_radius & 0xFF;
    buffer[19] = (calib->accel_radius >> 8) & 0xFF;
    buffer[20] = calib->mag_radius & 0xFF;
    buffer[21] = (calib->mag_radius >> 8) & 0xFF;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(bno->i2c, bno->address,
                                                  BNO055_ACCEL_OFFSET_X_LSB_ADDR, 1, buffer, 22, 30);
    if (status != HAL_OK)
        return status;

    return BNO055_SetMode(bno, saved_mode);
}

HAL_StatusTypeDef BNO055_GetCalibration(BNO055_t *bno, calibration_data_t *calib)
{
    uint8_t buffer[22];
    uint8_t saved_mode = bno->current_mode;

    if (BNO055_SetMode(bno, BNO055_MODE_CONFIG) != HAL_OK)
        return HAL_ERROR;

    if (ReadRegs(bno, BNO055_ACCEL_OFFSET_X_LSB_ADDR, buffer, 22) != HAL_OK)
        return HAL_ERROR;

    calib->accel_offset_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    calib->accel_offset_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    calib->accel_offset_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    calib->mag_offset_x = (int16_t)((buffer[7] << 8) | buffer[6]);
    calib->mag_offset_y = (int16_t)((buffer[9] << 8) | buffer[8]);
    calib->mag_offset_z = (int16_t)((buffer[11] << 8) | buffer[10]);
    calib->gyro_offset_x = (int16_t)((buffer[13] << 8) | buffer[12]);
    calib->gyro_offset_y = (int16_t)((buffer[15] << 8) | buffer[14]);
    calib->gyro_offset_z = (int16_t)((buffer[17] << 8) | buffer[16]);
    calib->accel_radius = (int16_t)((buffer[19] << 8) | buffer[18]);
    calib->mag_radius = (int16_t)((buffer[21] << 8) | buffer[20]);

    return BNO055_SetMode(bno, saved_mode);
}

HAL_StatusTypeDef BNO055_GetCalibrationStatus(BNO055_t *bno)
{
    uint8_t calib_stat;

    if (ReadReg(bno, BNO055_CALIB_STAT_ADDR, &calib_stat) != HAL_OK)
        return HAL_ERROR;

    bno->calib_status.system = (calib_stat >> 6) & 0x03;
    bno->calib_status.gyro = (calib_stat >> 4) & 0x03;
    bno->calib_status.accel = (calib_stat >> 2) & 0x03;
    bno->calib_status.mag = calib_stat & 0x03;

    bno->is_calibrated = (bno->calib_status.system >= 2 && bno->calib_status.mag >= 2);

    return HAL_OK;
}

bool BNO055_IsCalibrated(BNO055_t *bno)
{
    BNO055_GetCalibrationStatus(bno);
    return bno->is_calibrated;
}

bool BNO055_IsResponding(BNO055_t *bno)
{
    uint8_t chip_id;
    return (ReadReg(bno, BNO055_CHIP_ID_ADDR, &chip_id) == HAL_OK && chip_id == BNO055_ID);
}

HAL_StatusTypeDef BNO055_SoftReset(BNO055_t *bno)
{
    if (WriteReg(bno, BNO055_SYS_TRIGGER_ADDR, 0x20) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(650);

    bno->error_count = 0;
    return BNO055_Init(bno, bno->i2c, (bno->address == BNO055_ADDRESS_A) ? 0 : 1);
}

HAL_StatusTypeDef BNO055_Update(BNO055_t *bno)
{
    uint8_t buffer[45];

    if (ReadRegs(bno, BNO055_ACCEL_DATA_X_LSB_ADDR, buffer, 45) != HAL_OK) {
        if (bno->error_count > 10) {
            BNO055_SoftReset(bno);
        }
        return HAL_ERROR;
    }

    int16_t ax = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t ay = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t az = (int16_t)((buffer[5] << 8) | buffer[4]);
    bno->accel.x = ax / 100.0f;
    bno->accel.y = ay / 100.0f;
    bno->accel.z = az / 100.0f;

    int16_t mx = (int16_t)((buffer[7] << 8) | buffer[6]);
    int16_t my = (int16_t)((buffer[9] << 8) | buffer[8]);
    int16_t mz = (int16_t)((buffer[11] << 8) | buffer[10]);
    bno->mag.x = mx / 16.0f;
    bno->mag.y = my / 16.0f;
    bno->mag.z = mz / 16.0f;

    int16_t gx = (int16_t)((buffer[13] << 8) | buffer[12]);
    int16_t gy = (int16_t)((buffer[15] << 8) | buffer[14]);
    int16_t gz = (int16_t)((buffer[17] << 8) | buffer[16]);
    bno->gyro.x = gx / 900.0f;
    bno->gyro.y = gy / 900.0f;
    bno->gyro.z = gz / 900.0f;

    int16_t yaw = (int16_t)((buffer[19] << 8) | buffer[18]);
    int16_t roll = (int16_t)((buffer[21] << 8) | buffer[20]);
    int16_t pitch = (int16_t)((buffer[23] << 8) | buffer[22]);
    bno->euler.yaw = yaw / 900.0f;
    bno->euler.roll = roll / 900.0f;
    bno->euler.pitch = pitch / 900.0f;

    int16_t qw = (int16_t)((buffer[25] << 8) | buffer[24]);
    int16_t qx = (int16_t)((buffer[27] << 8) | buffer[26]);
    int16_t qy = (int16_t)((buffer[29] << 8) | buffer[28]);
    int16_t qz = (int16_t)((buffer[31] << 8) | buffer[30]);
    const float scale = 1.0f / 16384.0f;
    bno->quat.w = qw * scale;
    bno->quat.x = qx * scale;
    bno->quat.y = qy * scale;
    bno->quat.z = qz * scale;

    bno->temperature = (int8_t)buffer[44];

    bno->last_update = HAL_GetTick();
    bno->error_count = 0;

    return HAL_OK;
}

HAL_StatusTypeDef BNO055_UpdateDMA(BNO055_t *bno)
{
    if (!bno->dma_ready)
        return HAL_BUSY;

    bno->dma_ready = false;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(bno->i2c, bno->address,
                                                     BNO055_ACCEL_DATA_X_LSB_ADDR, 1, bno->dma_buffer, 45);
    if (status != HAL_OK) {
        bno->dma_ready = true;
        bno->error_count++;
        if (bno->error_count > 10) {
            BNO055_SoftReset(bno);
        }
    }
    return status;
}

void BNO055_ProcessDMA(BNO055_t *bno)
{
    uint8_t *b = bno->dma_buffer;

    bno->accel.x = ((int16_t)((b[1] << 8) | b[0])) / 100.0f;
    bno->accel.y = ((int16_t)((b[3] << 8) | b[2])) / 100.0f;
    bno->accel.z = ((int16_t)((b[5] << 8) | b[4])) / 100.0f;

    bno->mag.x = ((int16_t)((b[7] << 8) | b[6])) / 16.0f;
    bno->mag.y = ((int16_t)((b[9] << 8) | b[8])) / 16.0f;
    bno->mag.z = ((int16_t)((b[11] << 8) | b[10])) / 16.0f;

    bno->gyro.x = ((int16_t)((b[13] << 8) | b[12])) / 900.0f;
    bno->gyro.y = ((int16_t)((b[15] << 8) | b[14])) / 900.0f;
    bno->gyro.z = ((int16_t)((b[17] << 8) | b[16])) / 900.0f;

    bno->euler.yaw = ((int16_t)((b[19] << 8) | b[18])) / 900.0f;
    bno->euler.roll = ((int16_t)((b[21] << 8) | b[20])) / 900.0f;
    bno->euler.pitch = ((int16_t)((b[23] << 8) | b[22])) / 900.0f;

    const float s = 1.0f / 16384.0f;
    bno->quat.w = ((int16_t)((b[25] << 8) | b[24])) * s;
    bno->quat.x = ((int16_t)((b[27] << 8) | b[26])) * s;
    bno->quat.y = ((int16_t)((b[29] << 8) | b[28])) * s;
    bno->quat.z = ((int16_t)((b[31] << 8) | b[30])) * s;

    bno->temperature = (int8_t)b[44];
    bno->dma_ready = true;
    bno->last_update = HAL_GetTick();
    bno->error_count = 0;
}
