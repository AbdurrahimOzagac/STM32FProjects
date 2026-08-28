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

/**
 * @brief  TEST AMAÇLI: I2C1 (PB6=SCL, PB7=SDA) için sabit bus recovery fonksiyonu.
 */
void I2C1_BusRecovery_Test(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 1) PB6/PB7'yi Open-Drain GPIO çıkışına al */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_6; // SCL
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7; // SDA
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    /* 2) SCL'i 9 kez toggle'la */
    for (uint8_t i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
        {
            break; // SDA serbest kaldı, erken çık
        }
    }

    /* 3) Manuel STOP condition */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    /* 4) Pinleri I2C1 Alternate Function moduna geri döndür */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

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
	I2C1_BusRecovery_Test();
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
	return I2Csafe_Run(hi2c, HAL_I2C_Mem_Read, DevAddress, MemAddress, MemAddSize, pData, Size);

	//debug code
//	return HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, I2CSAFE_TIMEOUT);
}

/**
 * @brief  Wrapper for STM32 HAL I2C memory write function with safety checks.
 * @note   This function adapts the generic driver signature to STM32 HAL.
 */
int8_t I2C_Write_Safe(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {

	return I2Csafe_Run(hi2c, HAL_I2C_Mem_Write, DevAddress, MemAddress, MemAddSize, pData, Size);

	//debug code
//	return HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, I2CSAFE_TIMEOUT);
}
