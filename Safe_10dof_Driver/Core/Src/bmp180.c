/**
 ******************************************************************************
 * @file    bmp180.c
 * @brief    *
 * @details  ******************************************************************************
 */

#include "bmp180.h"

int8_t BMP180_Init(BMP180_Handle_t *handle, BMP180_I2C_Write_Fn write_fn,
		BMP180_I2C_Read_Fn read_fn, BMP180_GetTick_Fn tick_fn, void *ctx,
		uint32_t interval_ms) {

	if (handle == NULL || write_fn == NULL || read_fn == NULL
			|| tick_fn == NULL) {
		return -1;
	}

	handle->i2c_write = write_fn;
	handle->i2c_read = read_fn;
	handle->get_tick = tick_fn;
	handle->user_ctx = ctx;
	handle->interval_ms = interval_ms;

	handle->state = BMP180_STATE_IDLE;
	handle->busy = 0;
	handle->data_valid = 0;
	handle->filter_initialized = 0;
	handle->ref_captured = 0;

	int8_t status = BMP180_Read_Calibration(handle);
	if (status != 0) {
		return status;
	}

	BMP180_Start_Measurement(handle);

	return 0;
}

void BMP180_Poll(BMP180_Handle_t *handle) {

	switch (handle->state) {

	case BMP180_STATE_IDLE:

		if (handle->get_tick() - handle->timer >= handle->interval_ms) {
			//BMP180_Start_Measurement();
		}
		break;

	case BMP180_STATE_WAIT_TEMP:				//MAGIC STRING
		if (handle->get_tick() - handle->timer < 5)
			break;

		uint8_t raw_data[2];

		if (handle->i2c_read(BMP180_ADDR, BMP180_READ_REG, 1, raw_data, 2,
				handle->user_ctx) == HAL_OK){



		}

			break;

	case BMP180_STATE_WAIT_PRESS:
		//oss_interval depends on oss
		if (handle->get_tick() - handle->timer >= handle->oss_interval) {
		}

		break;

	}
}
