#ifndef DS3231_H
#define DS3231_H

#include <stdint.h>

/*
 * DS3231 RTC kutuphanesi - platformdan bagimsiz surum.
 *
 * Bu kutuphane herhangi bir HAL/MCU'ya dogrudan bagli degildir.
 * I2C okuma/yazma islemleri disaridan "fonksiyon isaretcisi" olarak
 * enjekte edilir. Boylece STM32, Arduino, ESP32, Linux i2c-dev vb.
 * her platformda sadece kucuk bir "port" katmani yazarak bu kutuphane
 * degistirilmeden kullanilabilir.
 */

#ifdef __cplusplus
extern "C" {
#endif

#define DS3231_I2C_ADDRESS   0x68U

typedef struct {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t dayOfWeek;
	uint8_t dayOfMonth;
	uint8_t month;
	uint8_t year;
} DS3231_Time_t;


typedef int8_t (*DS3231_I2C_Write_Fn)(uint8_t dev_addr, uint8_t reg_addr,
		const uint8_t *data, uint16_t len, void *user_ctx);

typedef int8_t (*DS3231_I2C_Read_Fn)(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len, void *user_ctx);

typedef struct {
	DS3231_I2C_Write_Fn i2c_write;
	DS3231_I2C_Read_Fn i2c_read;
	void *user_ctx; /* platforma ozel handle, orn. I2C_HandleTypeDef* */
} DS3231_Handle_t;

void DS3231_Init(DS3231_Handle_t *handle, DS3231_I2C_Write_Fn write_fn,
		DS3231_I2C_Read_Fn read_fn, void *user_ctx);

/* Basari: 0, Hata: 0'dan farkli (port katmanindan donen kod) */
int8_t DS3231_Set_Time(DS3231_Handle_t *handle, uint8_t sec, uint8_t min,
		uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month,
		uint8_t year);

/* Basari: 0, Hata: 0'dan farkli. Basarisizlikta *time icerigi degismez. */
int8_t DS3231_Get_Time(DS3231_Handle_t *handle, DS3231_Time_t *time);

#ifdef __cplusplus
}
#endif

#endif /* DS3231_H */

