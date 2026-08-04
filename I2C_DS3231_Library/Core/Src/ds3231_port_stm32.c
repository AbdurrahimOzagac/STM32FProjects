/**
 ******************************************************************************
 * @file    ds3231_port_stm32.c
 * @brief   STM32 HAL port layer implementation for the DS3231 driver.
 *
 * @details This is the only translation unit in the project that calls
 *          HAL_I2C_Mem_Write / HAL_I2C_Mem_Read directly. It adapts the
 *          platform-independent DS3231_I2C_Write_Fn / DS3231_I2C_Read_Fn
 *          signatures to STM32 HAL's calling convention.
 ******************************************************************************
 */

#include "ds3231_port_stm32.h"

/**
 * @brief HAL-backed implementation of DS3231_I2C_Write_Fn.
 *
 * @note HAL_I2C_Mem_Write() expects DevAddress in 8-bit form (7-bit
 *       address shifted left by one). @p dev_addr arrives here as the
 *       raw 7-bit address, so the shift is applied here — this is the
 *       single point in the codebase where that conversion happens.
 *
 * @param[in] dev_addr  7-bit I2C device address.
 * @param[in] reg_addr  Register address to start writing at.
 * @param[in] data      Bytes to write.
 * @param[in] len       Number of bytes to write.
 * @param[in] user_ctx  I2C_HandleTypeDef* for the target bus, as
 *                      registered by DS3231_Port_STM32_Init().
 *
 * @return 0 on HAL_OK, -1 otherwise.
 */
static int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr,
		const uint8_t *data, uint16_t len, void *user_ctx) {
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) user_ctx;
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Write(hi2c, (uint16_t) (dev_addr << 1), reg_addr, 1,
			(uint8_t *) data, len, HAL_MAX_DELAY);

	return (status == HAL_OK) ? 0 : -1;
}

/**
 * @brief HAL-backed implementation of DS3231_I2C_Read_Fn.
 *
 * @note See stm32_i2c_write() regarding the 7-bit -> 8-bit address shift.
 *
 * @param[in]  dev_addr  7-bit I2C device address.
 * @param[in]  reg_addr  Register address to start reading from.
 * @param[out] data      Buffer to receive the bytes read.
 * @param[in]  len       Number of bytes to read.
 * @param[in]  user_ctx  I2C_HandleTypeDef* for the target bus, as
 *                       registered by DS3231_Port_STM32_Init().
 *
 * @return 0 on HAL_OK, -1 otherwise.
 */
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
