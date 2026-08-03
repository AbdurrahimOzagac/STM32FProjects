#ifndef DS3231_H
#define DS3231_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define DS3231_WRITE_ADDRESS    0xD0
#define DS3231_READ_ADDRESS     0xD1

typedef struct {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t dayOfWeek;
	uint8_t dayOfMonth;
	uint8_t month;
	uint8_t year;
} DS3231_Time_t;

void DS3231_Set_Time(I2C_HandleTypeDef *hi2c, uint8_t sec, uint8_t min,
		uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month,
		uint8_t year);

DS3231_Time_t DS3231_Get_Time(I2C_HandleTypeDef *hi2c);

#endif
