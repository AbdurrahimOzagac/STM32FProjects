/**
 ******************************************************************************
 * @file    mpu6050.c
 * @author  abdurrahimozagac
 * @brief   MPU6050 6-Axis Accelerometer and Gyroscope Driver Implementation
 ******************************************************************************
 */

#include "mpu6050.h"

int8_t MPU6050_I2C_Init(MPU6050_Handle_t *handle, MPU6050_I2C_Write_Fn write_fn,
        MPU6050_I2C_Read_Fn read_fn, void *user_ctx) {

    handle->i2c_write = write_fn;
    handle->i2c_read = read_fn;
    handle->user_ctx = user_ctx;

    uint8_t chip_id = 0;

    // Check device identity (WHO_AM_I register)
    int8_t status = handle->i2c_read(MPU6050_ADDR, MPU6050_WHOAMI, &chip_id, MPU6050_CMD_LEN,
            handle->user_ctx);

    if (status != MPU6050_OK) return status;

    if (chip_id != MPU6050_EXPECTED_WHOAMI) return MPU6050_ERR_WHOAMI;

    // Apply default sensor configuration upon successful initialization
    return MPU6050_Set_Mode_Default(handle);
}

int8_t MPU6050_Set_Mode(MPU6050_Handle_t *handle, uint8_t wake, uint8_t accel_mode, uint8_t gyro_mode,
        uint8_t dlpf_mode) {

    uint8_t status = MPU6050_OK;

    // Configure Power Management (Wake up device)
    status = handle->i2c_write(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, &wake, MPU6050_CMD_LEN,
            handle->user_ctx);
    if (status != MPU6050_OK)
        return status;

    // Configure Accelerometer Full Scale Range
    status = handle->i2c_write(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, &accel_mode, MPU6050_CMD_LEN,
            handle->user_ctx);
    if (status != MPU6050_OK)
        return status;

    // Configure Gyroscope Full Scale Range
    status = handle->i2c_write(MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, &gyro_mode, MPU6050_CMD_LEN,
            handle->user_ctx);
    if (status != MPU6050_OK)
        return status;

    // Configure Digital Low Pass Filter (DLPF)
    status = handle->i2c_write(MPU6050_ADDR, MPU6050_REG_CONFIG, &dlpf_mode, MPU6050_CMD_LEN,
            handle->user_ctx);
    if (status != MPU6050_OK)
        return status;

    return MPU6050_OK;
}

int8_t MPU6050_Set_Mode_Default(MPU6050_Handle_t *handle) {
    // Default Mode: Wake, Accel (+-2g), Gyro (+-250dps), DLPF (~42Hz)
    return MPU6050_Set_Mode(handle, MPU6050_WAKE_UP_VAL, MPU6050_ACCEL_2G_VAL, MPU6050_GYRO_250DPS_VAL, MPU6050_DLPF_42HZ_VAL);
}

int8_t MPU6050_Get_Acceleration_Data(MPU6050_Handle_t *handle,
        MPU6050_Acceleration_t *out_acc) {

    uint8_t status = MPU6050_OK;
    uint8_t buffer[MPU6050_DATA_LEN_SINGLE] = {0};

    // Burst read 6 bytes starting from ACCEL_XOUT_H
    status = handle->i2c_read(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, buffer,
            MPU6050_DATA_LEN_SINGLE, handle->user_ctx);
    if (status != MPU6050_OK)
        return status;

    out_acc->accel_x = (int16_t) (buffer[0] << 8) | buffer[1];
    out_acc->accel_y = (int16_t) (buffer[2] << 8) | buffer[3];
    out_acc->accel_z = (int16_t) (buffer[4] << 8) | buffer[5];

    return MPU6050_OK;
}

int8_t MPU6050_Get_Gyroscope_Data(MPU6050_Handle_t *handle,
        Gyroscope_t *out_gyro) {

    uint8_t status = MPU6050_OK;
    uint8_t buffer[MPU6050_DATA_LEN_SINGLE] = {0};

    // Burst read 6 bytes starting from GYRO_XOUT_H
    status = handle->i2c_read(MPU6050_ADDR, MPU6050_REG_GYRO_XOUT_H, buffer,
            MPU6050_DATA_LEN_SINGLE, handle->user_ctx);

    if (status != MPU6050_OK)
        return status;

    out_gyro->gyro_x = (int16_t) (buffer[0] << 8) | buffer[1];
    out_gyro->gyro_y = (int16_t) (buffer[2] << 8) | buffer[3];
    out_gyro->gyro_z = (int16_t) (buffer[4] << 8) | buffer[5];

    return MPU6050_OK;
}

int8_t MPU6050_Get_Inertial_Data(MPU6050_Handle_t *handle,
        MPU6050_Inertial_t *out_inertia) {

    MPU6050_Acceleration_t accel;
    Gyroscope_t gyro;
    int8_t status = MPU6050_OK;
    uint8_t buffer[MPU6050_DATA_LEN_ALL] = {0};

    // Burst read 14 bytes starting from ACCEL_XOUT_H (Accel + Temp + Gyro)
    status = handle->i2c_read(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, buffer,
            MPU6050_DATA_LEN_ALL, handle->user_ctx);

    if (status != MPU6050_OK)
        return status;

    // Parse Acceleration Data
    accel.accel_x = (int16_t) (buffer[0] << 8) | buffer[1];
    accel.accel_y = (int16_t) (buffer[2] << 8) | buffer[3];
    accel.accel_z = (int16_t) (buffer[4] << 8) | buffer[5];

    // Parse Gyroscope Data (Skip buffer[6] and buffer[7] which hold Temperature)
    gyro.gyro_x = (int16_t) (buffer[8] << 8) | buffer[9];
    gyro.gyro_y = (int16_t) (buffer[10] << 8) | buffer[11];
    gyro.gyro_z = (int16_t) (buffer[12] << 8) | buffer[13];

    out_inertia->accel = accel;
    out_inertia->gyro = gyro;

    return MPU6050_OK;
}
