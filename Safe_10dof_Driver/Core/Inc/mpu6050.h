/**
 ******************************************************************************
 * @file
 * @brief
 *
 * @details
 *
 ******************************************************************************
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR		0x68
#define MPU6050_WHOAMI		0x75

typedef struct {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
} MPU6050_Acceleration_t;

typedef struct {
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} Gyroscope_t;


typedef int8_t (*MPU6050_I2C_Write_Fn)(uint8_t dev_addr, uint8_t reg_addr,
		const uint8_t *data, uint16_t len, void *user_ctx);

typedef int8_t (*MPU6050_I2C_Read_Fn)(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len, void *user_ctx);

typedef struct {
	MPU6050_Acceleration_t accel;
	Gyroscope_t gyro;
} MPU6050_Inertial_t; //Inertial data = Acceleration + Gyroscope data

typedef struct {
	MPU6050_I2C_Write_Fn i2c_write;
	MPU6050_I2C_Read_Fn i2c_read;
	void *user_ctx;

} MPU6050_Handle_t;


int8_t MPU6050_I2C_Init(MPU6050_Handle_t *handle, MPU6050_I2C_Write_Fn write_fn,
		MPU6050_I2C_Read_Fn read_fn, void *user_ctx);

int8_t MPU6050_Set_Mode(MPU6050_Handle_t *handle, uint8_t wake, uint8_t accel_mode, uint8_t gyro_mode,
		uint8_t dlpf_mode);

int8_t MPU6050_Set_Mode_Default(MPU6050_Handle_t *handle);

int8_t MPU6050_Get_Acceleration_Data(MPU6050_Handle_t *handle, MPU6050_Acceleration_t *out_acc);
int8_t MPU6050_Get_Gyroscope_Data(MPU6050_Handle_t *handle, Gyroscope_t *out_gyro);
int8_t MPU6050_Get_Inertial_Data(MPU6050_Handle_t *handle, MPU6050_Inertial_t *out_inertia);


#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
