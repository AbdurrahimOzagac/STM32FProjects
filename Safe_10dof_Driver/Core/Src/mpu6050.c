/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief    *
 * @details  ******************************************************************************
 */

#include "mpu6050.h"

int8_t MPU6050_I2C_Init(MPU6050_Handle_t *handle, MPU6050_I2C_Write_Fn write_fn,
		MPU6050_I2C_Read_Fn read_fn, void *user_ctx) {

	handle->i2c_write = write_fn;
	handle->i2c_read = read_fn;
	handle->user_ctx = user_ctx;

	uint8_t chip_id = 0;

	//whoami
	int8_t status = handle->i2c_read(MPU6050_ADDR, MPU6050_WHOAMI, &chip_id, 1,
			handle->user_ctx);

	if (status != 0 ) return status;

	if(chip_id != 0x68) return -2;

	return MPU6050_Set_Mode_Default(handle);
}

int8_t MPU6050_Set_Mode(MPU6050_Handle_t *handle, uint8_t wake, uint8_t accel_mode, uint8_t gyro_mode,
		uint8_t dlpf_mode){

	uint8_t status = 0;

	status = handle->i2c_write(MPU6050_ADDR, 0x6B, &wake, 1,
			handle->user_ctx);
	if (status != 0)
		return status;

	status = handle->i2c_write(MPU6050_ADDR, 0x1C, &accel_mode, 1,
			handle->user_ctx);
	if (status != 0)
		return status;

	status = handle->i2c_write(MPU6050_ADDR, 0x1B, &gyro_mode, 1,
			handle->user_ctx);
	if (status != 0)
		return status;

	status = handle->i2c_write(MPU6050_ADDR, 0x1A, &dlpf_mode, 1,
			handle->user_ctx);
	if (status != 0)
		return status;


	return 0;
}

int8_t MPU6050_Set_Mode_Default(MPU6050_Handle_t *handle){

	// Wake: 0x00, Accel (+-2g): 0x00, Gyro (+-250dps): 0x00, DLPF (~42Hz): 0x03
	return MPU6050_Set_Mode(handle,0x00, 0x00, 0x00, 0x03);

}

int8_t MPU6050_Get_Acceleration_Data(MPU6050_Handle_t *handle,
		MPU6050_Acceleration_t *out_acc){

	uint8_t status = 0;

	uint8_t buffer[6] = {0};

	status = handle->i2c_read(MPU6050_ADDR, 0x3B, buffer,
			6, handle->user_ctx);
	if (status != 0)
		return status;

	out_acc->accel_x = (int16_t) (buffer[0] << 8) | buffer[1];
	out_acc->accel_y = (int16_t) (buffer[2] << 8) | buffer[3];
	out_acc->accel_z = (int16_t) (buffer[4] << 8) | buffer[5];

	return 0;
}

int8_t MPU6050_Get_Gyroscope_Data(MPU6050_Handle_t *handle,
		Gyroscope_t *out_gyro){

	uint8_t status = 0;

	uint8_t buffer[6] = {0};

	status = handle->i2c_read(MPU6050_ADDR, 0x43, buffer,
			6, handle->user_ctx);

	if (status != 0)
		return status;

	out_gyro->gyro_x = (int16_t) (buffer[0] << 8) | buffer[1];
	out_gyro->gyro_y = (int16_t) (buffer[2] << 8) | buffer[3];
	out_gyro->gyro_z = (int16_t) (buffer[4] << 8) | buffer[5];

	return 0;
}

int8_t MPU6050_Get_Inertial_Data(MPU6050_Handle_t *handle,
		MPU6050_Inertial_t *out_inertia){

	MPU6050_Acceleration_t accel;
	Gyroscope_t gyro;

	uint8_t status = 0;

	uint8_t buffer[14] = {0};

	status = handle->i2c_read(MPU6050_ADDR, 0x3B, buffer,
			14, handle->user_ctx);

	if (status != 0)
		return status;

	accel.accel_x = (int16_t) (buffer[0] << 8) | buffer[1];
	accel.accel_y = (int16_t) (buffer[2] << 8) | buffer[3];
	accel.accel_z = (int16_t) (buffer[4] << 8) | buffer[5];

	gyro.gyro_x = (int16_t) (buffer[8] << 8) | buffer[9];
	gyro.gyro_y = (int16_t) (buffer[10] << 8) | buffer[11];
	gyro.gyro_z = (int16_t) (buffer[12] << 8) | buffer[13];

	out_inertia->accel = accel;
	out_inertia->gyro = gyro;

	return 0;
}
