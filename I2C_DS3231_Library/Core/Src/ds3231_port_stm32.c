#include "ds3231_port_stm32.h"

/*
 * HAL_I2C_Mem_Write/Read fonksiyonlari DevAddress parametresini
 * 8-bit (yani 7-bit adres << 1) olarak bekler. dev_addr buraya
 * 7-bit (0x68) olarak geldigi icin burada shiftliyoruz. Adres
 * donusumunun tek yapildigi yer burasi.
 */

static int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr,
		const uint8_t *data, uint16_t len, void *user_ctx) {
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) user_ctx;
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Write(hi2c, (uint16_t) (dev_addr << 1), reg_addr, 1,
			(uint8_t *) data, len, HAL_MAX_DELAY);

	return (status == HAL_OK) ? 0 : -1;
}

static int8_t stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len, void *user_ctx) {
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) user_ctx;
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(hi2c, (uint16_t) (dev_addr << 1), reg_addr, 1,
			data, len, HAL_MAX_DELAY);

	return (status == HAL_OK) ? 0 : -1;
}

int8_t DS3231_Port_STM32_Init(DS3231_Handle_t *handle, I2C_HandleTypeDef *hi2c) {
	if (handle == NULL || hi2c == NULL) {
		return -1;
	}

	DS3231_Init(handle, stm32_i2c_write, stm32_i2c_read, (void *) hi2c);

	return 0;
}