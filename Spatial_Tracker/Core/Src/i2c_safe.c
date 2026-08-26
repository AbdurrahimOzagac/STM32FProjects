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

#define I2CSAFE_TIMEOUT             100
#define I2CSAFE_RETRY_INTERVAL_MS   1
#define I2CSAFE_RESET_WAIT_MS       1

typedef enum {
    STATE_RUN, STATE_RETRY, STATE_RESET_INIT, STATE_RESET_WAIT
} I2C_State_t;

static I2C_State_t i2c_state = STATE_RUN;

static uint8_t retryCount = 0;
static uint32_t retryTime = 0;
static uint32_t resetTime = 0;

static int8_t I2Csafe_Run(I2C_HandleTypeDef *hi2c,
        HAL_StatusTypeDef (*HAL_I2C_Mem_Func)(I2C_HandleTypeDef *hi2c,
                uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
                uint8_t *pData, uint16_t Size, uint32_t Timeout),
        uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
        uint8_t *pData, uint16_t Size) {

    HAL_StatusTypeDef status;

    switch (i2c_state) {

    case STATE_RUN:

        status = HAL_I2C_Mem_Func(hi2c, DevAddress, MemAddress, MemAddSize,
                pData, Size, I2CSAFE_TIMEOUT);

        if (status == HAL_OK) {
            return 0;
        }

        else {
            i2c_state = STATE_RETRY;
            retryTime = HAL_GetTick();
            return -2;
        }
        break;

    case STATE_RETRY:

        if (HAL_GetTick() - retryTime < I2CSAFE_RETRY_INTERVAL_MS)
            return -2;

        retryTime = HAL_GetTick();
        if (retryCount < 3) {

            status = HAL_I2C_Mem_Func(hi2c, DevAddress, MemAddress, MemAddSize,
                    pData, Size, I2CSAFE_TIMEOUT);

            if (status == HAL_OK) {
                retryCount = 0;
                i2c_state = STATE_RUN;
                return 0;
            } else {
                retryCount++;
                return -2;
            }
        }

        else if (retryCount >= 3) {

            i2c_state = STATE_RESET_INIT;
            return -2;
        }
        break;

    case STATE_RESET_INIT:

        resetTime = HAL_GetTick();
        HAL_I2C_DeInit(hi2c);

        i2c_state = STATE_RESET_WAIT;

        return -2;

        break;

    case STATE_RESET_WAIT:

        if (HAL_GetTick() - resetTime < I2CSAFE_RETRY_INTERVAL_MS)
            return -2;


        HAL_I2C_Init(hi2c);

        retryCount=0;
        i2c_state = STATE_RUN;

        return -1;
    }

    return -2;
}

int8_t I2C_Read_Safe(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
        uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {

    return I2Csafe_Run(hi2c, HAL_I2C_Mem_Read, DevAddress, MemAddress,
            MemAddSize, pData, Size);

}

int8_t I2C_Write_Safe(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
        uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size) {

    return I2Csafe_Run(hi2c, HAL_I2C_Mem_Write, DevAddress, MemAddress,
            MemAddSize, pData, Size);
}
