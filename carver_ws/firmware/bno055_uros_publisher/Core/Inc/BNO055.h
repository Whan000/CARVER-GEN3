/*
 * BNO055.h
 *
 *  Created on: Aug 6, 2025
 *      Author: transporter
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define BNO055_ADDRESS_A    (0x28 << 1)
#define BNO055_ADDRESS_B    (0x29 << 1)
#define BNO055_ID           (0xA0)

#define BNO055_CHIP_ID_ADDR          0x00
#define BNO055_PAGE_ID_ADDR          0x07
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_MAG_DATA_X_LSB_ADDR   0x0E
#define BNO055_GYRO_DATA_X_LSB_ADDR  0x14
#define BNO055_EULER_H_LSB_ADDR      0x1A
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define BNO055_TEMP_ADDR              0x34
#define BNO055_CALIB_STAT_ADDR        0x35
#define BNO055_SYS_STAT_ADDR          0x39
#define BNO055_SYS_ERR_ADDR           0x3A
#define BNO055_UNIT_SEL_ADDR          0x3B
#define BNO055_OPR_MODE_ADDR          0x3D
#define BNO055_PWR_MODE_ADDR          0x3E
#define BNO055_SYS_TRIGGER_ADDR       0x3F
#define BNO055_AXIS_MAP_CONFIG_ADDR   0x41
#define BNO055_AXIS_MAP_SIGN_ADDR     0x42
#define BNO055_ACCEL_OFFSET_X_LSB_ADDR 0x55
#define BNO055_ACCEL_RADIUS_LSB_ADDR   0x67
#define BNO055_MAG_RADIUS_LSB_ADDR     0x69

#define BNO055_MODE_CONFIG    0x00
#define BNO055_MODE_NDOF      0x0C
#define BNO055_MODE_IMU       0x08
#define BNO055_MODE_COMPASS   0x09
#define BNO055_MODE_M4G       0x0A
#define BNO055_MODE_NDOF_FMC_OFF 0x0B

typedef enum {
    AXIS_REMAP_P0 = 0x21,
    AXIS_REMAP_P1 = 0x24,
    AXIS_REMAP_P2 = 0x24,
    AXIS_REMAP_P3 = 0x21,
    AXIS_REMAP_P4 = 0x24,
    AXIS_REMAP_P5 = 0x21,
    AXIS_REMAP_P6 = 0x21,
    AXIS_REMAP_P7 = 0x24
} axis_remap_config_t;

typedef enum {
    AXIS_REMAP_SIGN_P0 = 0x04,
    AXIS_REMAP_SIGN_P1 = 0x00,
    AXIS_REMAP_SIGN_P2 = 0x06,
    AXIS_REMAP_SIGN_P3 = 0x02,
    AXIS_REMAP_SIGN_P4 = 0x03,
    AXIS_REMAP_SIGN_P5 = 0x01,
    AXIS_REMAP_SIGN_P6 = 0x07,
    AXIS_REMAP_SIGN_P7 = 0x05
} axis_remap_sign_t;

typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} euler_t;

typedef struct {
    uint8_t system;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} calibration_status_t;

typedef struct {
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    int16_t mag_offset_z;
    int16_t accel_radius;
    int16_t mag_radius;
} calibration_data_t;

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t address;
    uint8_t current_mode;
    axis_remap_config_t current_remap_config;
    axis_remap_sign_t current_remap_sign;
    vector3_t accel;
    vector3_t gyro;
    vector3_t mag;
    euler_t euler;
    quaternion_t quat;
    calibration_status_t calib_status;
    int8_t temperature;
    bool is_calibrated;
    uint8_t dma_buffer[45];
    volatile bool dma_ready;
    uint32_t last_update;
    uint8_t error_count;
} BNO055_t;

HAL_StatusTypeDef BNO055_Init(BNO055_t *bno, I2C_HandleTypeDef *i2c, uint8_t addr);
HAL_StatusTypeDef BNO055_SetMode(BNO055_t *bno, uint8_t mode);
HAL_StatusTypeDef BNO055_SetAxisRemap(BNO055_t *bno, axis_remap_config_t config, axis_remap_sign_t sign);
HAL_StatusTypeDef BNO055_LoadCalibration(BNO055_t *bno, const calibration_data_t *calib);
HAL_StatusTypeDef BNO055_GetCalibration(BNO055_t *bno, calibration_data_t *calib);
HAL_StatusTypeDef BNO055_GetCalibrationStatus(BNO055_t *bno);
bool BNO055_IsCalibrated(BNO055_t *bno);
bool BNO055_IsResponding(BNO055_t *bno);
HAL_StatusTypeDef BNO055_SoftReset(BNO055_t *bno);
HAL_StatusTypeDef BNO055_Update(BNO055_t *bno);
HAL_StatusTypeDef BNO055_UpdateDMA(BNO055_t *bno);
void BNO055_ProcessDMA(BNO055_t *bno);

#endif /* INC_BNO055_H_ */
