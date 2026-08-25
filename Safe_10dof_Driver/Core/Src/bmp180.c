/**
 ******************************************************************************
 * @file    bmp180.c
 * @brief    *
 * @details  ******************************************************************************
 */

#include "bmp180.h"
#include <stddef.h>

static const uint8_t bmp_oss_delays[4] = { 5, 8, 14, 26 };

static int8_t BMP180_Read_Calibration(BMP180_Handle_t *handle);

static void BMP180_Get_Temperature_Celsius(uint16_t ut, BMP180_Calib_t *calib,
		long *out_b5_ref, float *out_tempc);

static void BMP180_Get_Pressure_Pa(uint32_t upressure, BMP180_Calib_t *calib,
		long *b5_ref, OSS oss, float *out_pressure_pa);

int8_t BMP180_Init(BMP180_Handle_t *handle, BMP180_I2C_Write_Fn write_fn,
		BMP180_I2C_Read_Fn read_fn, BMP180_GetTick_Fn tick_fn, void *ctx,
		uint32_t interval_ms) {

	if (handle == NULL || write_fn == NULL || read_fn == NULL || tick_fn == NULL) {
		return -1;
	}

	handle->i2c_write = write_fn;
	handle->i2c_read = read_fn;
	handle->get_tick = tick_fn;
	handle->user_ctx = ctx;

	handle->state = BMP180_STATE_WAIT_TEMP;

	int8_t status = BMP180_Read_Calibration(handle);
	if (status != 0)
		return status;

	uint8_t cmd = BMP180_COMMAND_TEMP;
	if (handle->i2c_write(BMP180_ADDR, BMP180_CONTROL_REG, &cmd, 1, ctx) != 0) {
		return -1;
	}

	handle->state = BMP180_STATE_WAIT_TEMP;
	handle->timer = handle->get_tick();

	return 0;

}

int8_t BMP180_Poll(BMP180_Handle_t *handle) {

	uint8_t command = 0;

	switch (handle->state) {
	case BMP180_STATE_WAIT_TEMP: {				//MAGIC STRING

		if (handle->get_tick() - handle->timer < 5) {
			return -2;
		}

		uint8_t temp_raw_data[2];

		//Read u_temperature data
		if (handle->i2c_read(BMP180_ADDR, BMP180_READ_REG, temp_raw_data, 2,
				handle->user_ctx) != 0) {
			return -1;
		}

		handle->data.utemp = (temp_raw_data[0] << 8) | temp_raw_data[1];

		//Write get pressure command
		command = BMP180_COMMAND_PRESS + (handle->oss << 6);
		if (handle->i2c_write(BMP180_ADDR, BMP180_CONTROL_REG, &command, 1,
				handle->user_ctx) != 0) {
			return -1;
		}

		handle->state = BMP180_STATE_WAIT_PRESS;
		handle->timer = handle->get_tick();

		return 0;
	}

	case BMP180_STATE_WAIT_PRESS: {
		//oss_interval depends on oss
		if (handle->get_tick() - handle->timer < bmp_oss_delays[handle->oss]) {
			return -2;
		}

		uint8_t pressure_raw_data[3];

		//Read u_pressure data
		if (handle->i2c_read(BMP180_ADDR, BMP180_READ_REG, pressure_raw_data, 3,
				handle->user_ctx) != 0) {
			return -1;
		}

		handle->data.upressure = (((uint32_t) pressure_raw_data[0] << 16)
				| ((uint32_t) pressure_raw_data[1] << 8)
				| ((uint32_t) pressure_raw_data[2])) >> (8 - handle->oss);

		BMP180_Get_Temperature_Celsius(handle->data.utemp, &handle->calib,
				&handle->_b5, &handle->data.temperature_c);
		BMP180_Get_Pressure_Pa(handle->data.upressure, &handle->calib,
				&handle->_b5, handle->oss, &handle->data.pressure_pa);

		//Write get temperature command
		command = BMP180_COMMAND_TEMP;
		if (handle->i2c_write(BMP180_ADDR, BMP180_CONTROL_REG, &command, 1,
				handle->user_ctx) != 0) {
			return -1;
		}

		handle->state = BMP180_STATE_WAIT_TEMP;
		handle->timer = handle->get_tick();

		handle->latest_atomic_data = handle->data;
		handle->data_valid = 1;

		return 0;
	}
	}
	return -1;
}

static int8_t BMP180_Read_Calibration(BMP180_Handle_t *handle) {

	uint8_t buffer[22];

	//Read all 22 calibration
	if (handle->i2c_read(BMP180_ADDR, 0xAA, buffer, 22, handle->user_ctx)
			!= 0) {
		return -1;
	}

	handle->calib.AC1 = (buffer[0] << 8) | buffer[1];
	handle->calib.AC2 = (buffer[2] << 8) | buffer[3];
	handle->calib.AC3 = (buffer[4] << 8) | buffer[5];
	handle->calib.AC4 = (buffer[6] << 8) | buffer[7];
	handle->calib.AC5 = (buffer[8] << 8) | buffer[9];
	handle->calib.AC6 = (buffer[10] << 8) | buffer[11];
	handle->calib.B1 = (buffer[12] << 8) | buffer[13];
	handle->calib.B2 = (buffer[14] << 8) | buffer[15];
	handle->calib.MB = (buffer[16] << 8) | buffer[17];
	handle->calib.MC = (buffer[18] << 8) | buffer[19];
	handle->calib.MD = (buffer[20] << 8) | buffer[21];

	return 0;
}

static void BMP180_Get_Temperature_Celsius(uint16_t ut, BMP180_Calib_t *calib,
		long *out_b5_ref, float *out_tempc) {
	long x1, x2;
	long temp_tenth = 0;

	x1 = (((long) ut - (long) calib->AC6) * (long) calib->AC5) >> 15;
	x2 = ((long) calib->MC << 11) / (x1 + (long) calib->MD);

	//Needed for pressure calibration
	*out_b5_ref = x1 + x2;

	temp_tenth = (*out_b5_ref + 8) >> 4;

	*out_tempc = temp_tenth / 10.0f;
}

static void BMP180_Get_Pressure_Pa(uint32_t upressure, BMP180_Calib_t *calib,
		long *b5_ref, OSS oss, float *out_pressure_pa) {
	long x1, x2, x3, b3, b6;
	unsigned long b4, b7, p;

	b6 = *b5_ref - 4000;

	x1 = ((long) calib->B2 * ((b6 * b6) >> 12)) >> 11;
	x2 = ((long) calib->AC2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((long) calib->AC1) * 4 + x3) << oss) + 2) >> 2;

	x1 = ((long) calib->AC3 * b6) >> 13;
	x2 = ((long) calib->B1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = ((long) calib->AC4 * (unsigned long) (x3 + 32768)) >> 15;

	b7 = ((unsigned long) upressure - b3) * (50000 >> oss);

	if (b7 < 0x80000000) {
		p = (b7 << 1) / b4;
	} else {
		p = (b7 / b4) << 1;
	}

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p = p + ((x1 + x2 + 3791) >> 4);

	*out_pressure_pa = (float) p;
}

int8_t BMP180_Get_Data(BMP180_Handle_t *handle, BMP180_Data_t *out_data) {

	if (handle == NULL || out_data == NULL || !handle->data_valid) {
		return -1;
	}

	*out_data = handle->latest_atomic_data;

	return 0;
}
