/**
 ******************************************************************************
 * @file
 * @brief
 *
 * @details
 *
 ******************************************************************************
 */

#ifndef QMC5883P_H
#define QMC5883P_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QMC5883P_ADDR	0x2CU

typedef struct {

	int16_t x_mag;
	int16_t y_mag;
	int16_t z_mag;

} QMC5883P_MAGNETOMETER_t;

typedef int8_t (*QMC5883P_I2C_Write_Fn)(uint8_t dev_addr, uint8_t reg_addr,
		const uint8_t *data, uint16_t len, void *user_ctx);

typedef int8_t (*QMC5883P_I2C_Read_Fn)(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len, void *user_ctx);

typedef struct {
	QMC5883P_I2C_Write_Fn i2c_write;
	QMC5883P_I2C_Read_Fn i2c_read;
	void *user_ctx;

} QMC5883P_Handle_t;

int8_t QMC5883P_I2C_Init(QMC5883P_Handle_t *handle, QMC5883P_I2C_Write_Fn write_fn,
		QMC5883P_I2C_Read_Fn read_fn, void *user_ctx);

int8_t QMC5883P_Set_Mode(QMC5883P_Handle_t *handle, uint8_t period_cfg,
		uint8_t set_reset, uint8_t ctrl_reg1);

// Period: 0x06, Set/Reset: 0x08, Ctrl1 (200Hz, +-30G, Cont.): 0xCF
int8_t QMC5883P_Set_Mode_Default(QMC5883P_Handle_t *handle);

int8_t QMC5883P_Get_Values(QMC5883P_Handle_t *handle, QMC5883P_MAGNETOMETER_t *out);

#ifdef __cplusplus
}
#endif

#endif /* QMC5883P_H */



