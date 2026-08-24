/**
 ******************************************************************************
 * @file
 * @brief
 *
 * @details
 *
 ******************************************************************************
 */

#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMP180_ADDR			0x77

#define BMP180_COMMAND_TEMP  	0x2E
#define BMP180_COMMAND_PRESS 	0x34

#define BMP180_CONTROL_REG	0xF4
#define BMP180_READ_REG		0xF6

#define BMP180_OSS         	0

//FILTER SHOULD BE EDITABLE
//#define BMP180_EMA_ALPHA_PRESS  	0.1f
//#define BMP180_EMA_ALPHA_TEMP   	0.1f

//SHOLD THINK THIS ALSO
#define PRESSURE_SEA_LEVEL_REFERANCE	105904.0f

typedef struct {
	int16_t AC1, AC2, AC3;
	uint16_t AC4, AC5, AC6;
	int16_t B1, B2, MB, MC, MD;
} BMP180_Calib_t;

typedef enum {
	BMP180_STATE_IDLE = 0, BMP180_STATE_WAIT_TEMP, BMP180_STATE_WAIT_PRESS
} BMP180_State_t;

typedef struct {
	BMP180_I2C_Write_Fn i2c_write;
	BMP180_I2C_Read_Fn i2c_read;
	BMP180_GetTick_Fn get_tick;
	void *user_ctx;

	BMP180_Calib_t calib;

	BMP180_State_t state;
	uint32_t timer;
	volatile uint8_t busy;
	long _b5;

	uint32_t interval_ms;

	float temp_c_filtered;
	float pressure_pa_filtered;
	uint8_t filter_initialized;

	float ref_pressure_pa;
	uint8_t ref_captured;

	BMP180_Data_t data;
	uint8_t data_valid;
} BMP180_Handle_t;

int8_t BMP180_Init(BMP180_Handle_t *handle, BMP180_I2C_Write_Fn write_fn,
		BMP180_I2C_Read_Fn read_fn, BMP180_GetTick_Fn tick_fn, void *ctx,
		uint32_t interval_ms);

void    BMP180_Poll(BMP180_Handle_t *h);

uint8_t BMP180_Get_Data(BMP180_Handle_t *h, BMP180_Data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BMP180_H */

