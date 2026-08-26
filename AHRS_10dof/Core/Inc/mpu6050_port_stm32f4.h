/**
 ******************************************************************************
 * @file    mpu6050_port_stm32f4.h
 * @author  abdurrahimozagac
 * @brief   STM32F4 Specific Port/Wrapper for MPU6050 Driver
 *
 * @details This file provides the function prototypes to link the generic
 *          MPU6050 IMU driver with the STM32 HAL I2C driver.
 ******************************************************************************
 */

#ifndef MPU6050_PORT_STM32_H
#define MPU6050_PORT_STM32_H

#include "mpu6050.h"
#include "stm32f4xx_hal.h"

/**
 * @brief  Initializes the MPU6050 handle using STM32 HAL I2C routines.
 * @param  handle: Pointer to the MPU6050 driver handle.
 * @param  hi2c:   Pointer to the STM32 HAL I2C handle (e.g., &hi2c1).
 * @retval MPU6050_OK on success, MPU6050_ERROR on pointer null error.
 */
int8_t MPU6050_Port_STM32_Init(MPU6050_Handle_t *handle, I2C_HandleTypeDef *hi2c);

#endif /* MPU6050_PORT_STM32_H */
