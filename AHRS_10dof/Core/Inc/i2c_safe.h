/**
 ******************************************************************************
 * @file
 * @brief
 *
 * @details
 *
 ******************************************************************************
 */

#ifndef I2C_SAFE_H
#define I2C_SAFE_H

#include <stdint.h>
#include "stm32f4xx_hal.h"


#ifdef __cplusplus
extern "C" {
#endif



int8_t I2C_Read_Safe(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

int8_t I2C_Write_Safe(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);


#ifdef __cplusplus
}
#endif

#endif /* I2C_SAFE_H */
