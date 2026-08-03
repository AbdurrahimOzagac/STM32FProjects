#include "ds3231.h"

uint8_t DEC_to_BCD(uint8_t value) {

	return (uint8_t) ((value / 10) * 16) + (value % 10);
}

uint8_t BCD_to_DEC(uint8_t value) {

	return (uint8_t) ((value / 16) * 10) + (value % 16);
}

void DS3231_Init(DS3231_Handle_t *handle, DS3231_I2C_Write_Fn write_fn,
		DS3231_I2C_Read_Fn read_fn, void *user_ctx) {
	handle->i2c_write = write_fn;
	handle->i2c_read = read_fn;
	handle->user_ctx = user_ctx;
}

int8_t DS3231_Set_Time(DS3231_Handle_t *handle, uint8_t sec, uint8_t min,
		uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month,
		uint8_t year) {
	uint8_t time_data[7] = { 0 };

	time_data[0] = DEC_to_BCD(sec);
	time_data[1] = DEC_to_BCD(min);
	time_data[2] = DEC_to_BCD(hour);
	time_data[3] = DEC_to_BCD(dayOfWeek);
	time_data[4] = DEC_to_BCD(dayOfMonth);
	time_data[5] = DEC_to_BCD(month);
	time_data[6] = DEC_to_BCD(year);

	return handle->i2c_write(DS3231_I2C_ADDRESS, 0x00, time_data, 7,
			handle->user_ctx);
}

int8_t DS3231_Get_Time(DS3231_Handle_t *handle, DS3231_Time_t *time) {
	uint8_t raw[7] = { 0 };
	int8_t status;

	status = handle->i2c_read(DS3231_I2C_ADDRESS, 0x00, raw, 7,
			handle->user_ctx);

	if (status == 0) {
		time->sec = BCD_to_DEC(raw[0] & 0x7F);
		time->min = BCD_to_DEC(raw[1] & 0x7F);
		time->hour = BCD_to_DEC(raw[2] & 0x3F);
		time->dayOfWeek = BCD_to_DEC(raw[3] & 0x07);
		time->dayOfMonth = BCD_to_DEC(raw[4] & 0x3F);
		time->month = BCD_to_DEC(raw[5] & 0x1F);
		time->year = BCD_to_DEC(raw[6]);
	}

	return status;
}

