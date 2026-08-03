#include "ds3231.h"

uint8_t DEC_to_BCD(uint8_t value) {

	return (uint8_t) ((value / 10) * 16) + (value % 10);
}

uint8_t BCD_to_DEC(uint8_t value) {

	return (uint8_t) ((value / 16) * 10) + (value % 16);
}

void DS3231_Set_Time(I2C_HandleTypeDef *hi2c, uint8_t sec, uint8_t min,
		uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month,
		uint8_t year) {
	//MAKE LIBRARY

	uint8_t time_data[7] = { 0 };

	time_data[0] = DEC_to_BCD(sec);
	time_data[1] = DEC_to_BCD(min);
	time_data[2] = DEC_to_BCD(hour);
	time_data[3] = DEC_to_BCD(dayOfWeek);
	time_data[4] = DEC_to_BCD(dayOfMonth);
	time_data[5] = DEC_to_BCD(month);
	time_data[6] = DEC_to_BCD(year);

	//MAKE LIBRARY HERE TOO TO BE INDEPENDENT TO HAL & DEVICE

	HAL_I2C_Mem_Write(hi2c, DS3231_WRITE_ADDRESS, 0x00, 1, time_data, 7,
	HAL_MAX_DELAY);
}

DS3231_Time_t DS3231_Get_Time(I2C_HandleTypeDef *hi2c) {
	DS3231_Time_t dsTime = { 0 };
	uint8_t receive_time[7] = { 0 };

	if (HAL_I2C_Mem_Read(hi2c, DS3231_READ_ADDRESS, 0x00, 1, receive_time, 7,
			HAL_MAX_DELAY) == HAL_OK) {
		dsTime.sec = BCD_to_DEC(receive_time[0]);
		dsTime.min = BCD_to_DEC(receive_time[1]);
		dsTime.hour = BCD_to_DEC(receive_time[2] & 0x3F);
		dsTime.dayOfWeek = BCD_to_DEC(receive_time[3]);
		dsTime.dayOfMonth = BCD_to_DEC(receive_time[4]);
		dsTime.month = BCD_to_DEC(receive_time[5]);
		dsTime.year = BCD_to_DEC(receive_time[6]);
	}
	return dsTime;
}

