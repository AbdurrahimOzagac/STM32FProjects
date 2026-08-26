/**
 ******************************************************************************
 * @file    i2c_safe.c
 * @brief   Safe I2C wrapper implementation with Circuit Breaker and Backoff mechanism.
 * @details This module wraps STM32 HAL I2C memory read/write functions to provide
 *          robust error handling, automatic bus reset, and device health tracking
 *          to prevent unresponsive peripherals from blocking the main control loop.
 *******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "i2c_safe.h"

#define I2CSAFE_TIMEOUT             100
#define I2CSAFE_RETRY_DELAY_MS      2
#define I2CSAFE_MAX_RETRIES         3
#define MAX_I2C_DEVICES             128

typedef struct {
	uint8_t fail_level;
	uint32_t next_try_time;
} I2C_Health_t;

static I2C_Health_t device_health[MAX_I2C_DEVICES] = { 0 };

static const uint32_t backoff_delays[] = { 0, 5, 20, 50, 200, 500, 1000 };
#define MAX_FAIL_LEVEL 6

/**
 * @brief  Core safe execution runner with circuit breaker and retry logic.
 */

static int8_t I2Csafe_Run(I2C_HandleTypeDef *hi2c,
		HAL_StatusTypeDef (*HAL_I2C_Mem_Func)(I2C_HandleTypeDef *hi2c,
				uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
				uint8_t *pData, uint16_t Size, uint32_t Timeout),
		uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
		uint8_t *pData, uint16_t Size) {

	HAL_StatusTypeDef status;
	uint8_t addr_index = (DevAddress >> 1) & 0x7F;

	if (device_health[addr_index].fail_level > 0) {
		if (HAL_GetTick() < device_health[addr_index].next_try_time) {
			return -2;
		}
	}

	for (uint8_t i = 0; i < I2CSAFE_MAX_RETRIES; i++) {
		status = HAL_I2C_Mem_Func(hi2c, DevAddress, MemAddress, MemAddSize,
				pData, Size, I2CSAFE_TIMEOUT);

		if (status == HAL_OK) {
			device_health[addr_index].fail_level = 0;
			return 0;
		}

		HAL_Delay(I2CSAFE_RETRY_DELAY_MS);
	}

	HAL_I2C_DeInit(hi2c);
	HAL_Delay(I2CSAFE_RETRY_DELAY_MS);
	HAL_I2C_Init(hi2c);

	if (device_health[addr_index].fail_level < MAX_FAIL_LEVEL) {
		device_health[addr_index].fail_level++;
	}

	device_health[addr_index].next_try_time = HAL_GetTick()
			+ backoff_delays[device_health[addr_index].fail_level];

	return -2;
}

/**
 * @brief  Wrapper for STM32 HAL I2C memory read function with safety checks.
 * @note   This function adapts the generic driver signature to STM32 HAL.
 */
int8_t I2C_Read_Safe(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {
	return I2Csafe_Run(hi2c, HAL_I2C_Mem_Read, DevAddress, MemAddress,
			MemAddSize, pData, Size);
}

/**
 * @brief  Wrapper for STM32 HAL I2C memory write function with safety checks.
 * @note   This function adapts the generic driver signature to STM32 HAL.
 */
int8_t I2C_Write_Safe(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {

	return I2Csafe_Run(hi2c, HAL_I2C_Mem_Write, DevAddress, MemAddress,
			MemAddSize, pData, Size);
}
