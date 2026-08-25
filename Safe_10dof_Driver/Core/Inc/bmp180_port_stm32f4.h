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


#ifndef BMP180_PORT_STM32_H
#define BMP180_PORT_STM32_H

#include "bmp180.h"

#include "stm32f4xx_hal.h"
int8_t BMP180_Port_STM32_Init(BMP180_Handle_t *handle, I2C_HandleTypeDef *hi2c);

#endif /* BMP180_PORT_STM32_H */
