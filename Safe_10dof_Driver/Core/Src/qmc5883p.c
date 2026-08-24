/**
 ******************************************************************************
 * @file    qmc5883p.c
 * @brief    *
 * @details  ******************************************************************************
 */

#include "qmc5883p.h"

int8_t QMC5883P_I2C_Init(QMC5883P_Handle_t *handle,
		QMC5883P_I2C_Write_Fn write_fn, QMC5883P_I2C_Read_Fn read_fn,
		void *user_ctx) {
	handle->i2c_write = write_fn;
	handle->i2c_read = read_fn;
	handle->user_ctx = user_ctx;

	uint8_t chip_id = 0;

	//whoami
	int8_t status = handle->i2c_read(QMC5883P_ADDR, QMC5883P_WHOAMI, &chip_id, 1,
			handle->user_ctx);

	if (status != 0 ) return status;

	if(chip_id != 0x80) return -2;

	return QMC5883P_Set_Mode_Default(handle);

}

int8_t QMC5883P_Set_Mode(QMC5883P_Handle_t *handle, uint8_t period_cfg,
		uint8_t set_reset, uint8_t ctrl_reg1) {

	int8_t status = 0;

	status = handle->i2c_write(QMC5883P_ADDR, 0x29, &period_cfg, 1,
			handle->user_ctx);
	if (status != 0)
		return status;

	status = handle->i2c_write(QMC5883P_ADDR, 0x0B, &set_reset, 1,
			handle->user_ctx);
	if (status != 0)
		return status;

	status = handle->i2c_write(QMC5883P_ADDR, 0x0A, &ctrl_reg1, 1,
			handle->user_ctx);
	if (status != 0)
		return status;

	return 0;
}

// Period: 0x06, Set/Reset: 0x08, Ctrl1 (200Hz, +-30G, Cont.): 0xCF //DETAYLANDILILABİİLR
int8_t QMC5883P_Set_Mode_Default(QMC5883P_Handle_t *handle) {
	return QMC5883P_Set_Mode(handle, 0x06, 0x08, 0xCF);
}

int8_t QMC5883P_Get_Magnetic_Data(QMC5883P_Handle_t *handle,
		QMC5883P_MAGNETOMETER_t *out_mag) {

	uint8_t status = 0;

	uint8_t buffer[6] = { 0 };

	status = handle->i2c_read(QMC5883P_ADDR, 0x01, buffer, 6, handle->user_ctx);

	if (status != 0)
		return status;

	out_mag->x_mag = (int16_t) ((buffer[1] << 8) | buffer[0]);
	out_mag->y_mag = (int16_t) ((buffer[3] << 8) | buffer[2]);
	out_mag->z_mag = (int16_t) ((buffer[5] << 8) | buffer[4]);

	return 0;
}

//Kalibrasyon fonksiyonları
