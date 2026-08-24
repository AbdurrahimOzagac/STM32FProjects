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


#ifndef MPU6050_PORT_STM32_H
#define MPU6050_PORT_STM32_H

#include "mpu6050.h"
#include "stm32f4xx_hal.h"

int8_t MPU6050_Port_STM32_Init(MPU6050_Handle_t *handle, I2C_HandleTypeDef *hi2c);

#endif /* MPU6050_PORT_STM32_H */
