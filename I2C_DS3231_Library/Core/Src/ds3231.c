/**
 ******************************************************************************
 * @file    ds3231.c
 * @brief   Platform-independent driver implementation for the DS3231 RTC.
 *
 * @details This file contains the device logic only (BCD conversion,
 *          register layout, time struct handling). It never touches any
 *          vendor HAL directly; all bus I/O goes through the callbacks
 *          stored in DS3231_Handle_t, injected via DS3231_Init().
 ******************************************************************************
 */

#include "ds3231.h"

/**
 * @brief Convert an 8-bit decimal value to its BCD (Binary Coded Decimal)
 *        representation.
 *
 * The DS3231 stores time fields internally as BCD, where the upper nibble
 * holds the tens digit and the lower nibble holds the units digit
 * (e.g. decimal 45 -> 0x45).
 *
 * @param[in] value Decimal value (expected range depends on the field,
 *                  e.g. 0-59 for seconds).
 * @return BCD-encoded byte.
 */
static uint8_t DEC_to_BCD(uint8_t value) {
	return (uint8_t) (((value / 10) * 16) + (value % 10));
}

/**
 * @brief Convert a BCD-encoded byte back to its 8-bit decimal value.
 *
 * @param[in] value BCD-encoded byte read from a DS3231 register.
 * @return Decimal value.
 */
static uint8_t BCD_to_DEC(uint8_t value) {
	return (uint8_t) (((value / 16) * 10) + (value % 16));
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
	/* DS3231 register map (0x00-0x06), written sequentially starting
	 * from register 0x00 thanks to the chip's internal address
	 * auto-increment feature. */
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
		/* Bit masks strip control/status bits that share a register
		 * with a time field:
		 *   - raw[5] bit 7 is the century flag, not part of the month
		 *     value, and must be cleared before BCD conversion.
		 *   - The remaining masks defensively clear reserved bits
		 *     that the datasheet does not guarantee to read as zero. */
		time->sec = BCD_to_DEC(raw[0] & 0x7F);
		time->min = BCD_to_DEC(raw[1] & 0x7F);
		time->hour = BCD_to_DEC(raw[2] & 0x3F);
		time->dayOfWeek = BCD_to_DEC(raw[3] & 0x07);
		time->dayOfMonth = BCD_to_DEC(raw[4] & 0x3F);
		time->month = BCD_to_DEC(raw[5] & 0x1F);
		time->year = BCD_to_DEC(raw[6]);
	}
	/* On failure, *time is intentionally left untouched so the caller
	 * never observes a partially-updated or garbage timestamp. */

	return status;
}
