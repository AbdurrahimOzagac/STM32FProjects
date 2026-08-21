/**
 ******************************************************************************
 * @file
 * @brief
 * @details
 *
 *
 *
 *
 *******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "i2c_safe.h"

#define SAFE_I2C_TIMEOUT 100

static void I2C_Reset(I2C_HandleTypeDef *hi2c) {

	HAL_I2C_DeInit(hi2c);
	HAL_Delay(1);
	HAL_I2C_Init(hi2c);
}

int8_t I2C_Read_Safe(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {

	HAL_StatusTypeDef status;

	for (uint8_t retry = 0; retry < 3; retry++) {
		status = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize,
				pData, Size, SAFE_I2C_TIMEOUT);

		if (status != HAL_OK) {
			return 0;
		}
		HAL_Delay(1);
	}

	I2C_Reset(hi2c);

	return -1;
}

int8_t I2C_Write_Safe(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {
	HAL_StatusTypeDef status;

	for (uint8_t retry = 0; retry < 3; retry++) {
		status = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize,
				pData, Size, SAFE_I2C_TIMEOUT);

		if (status != HAL_OK) {
			return 0;
		}
		HAL_Delay(1);
	}

	I2C_Reset(hi2c);

	return -1;
}

