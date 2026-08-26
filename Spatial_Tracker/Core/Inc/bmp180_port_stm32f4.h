/**
 ******************************************************************************
 * @file    bmp180_port_stm32f4.h
 * @author  abdurrahimozagac
 * @brief   STM32F4 Specific Port/Wrapper for BMP180 Driver
 *
 * @details This file provides the function prototypes to link the generic
 *          BMP180 driver with the STM32 HAL I2C and System Tick routines.
 ******************************************************************************
 */

#ifndef BMP180_PORT_STM32_H
#define BMP180_PORT_STM32_H

#include "bmp180.h"
#include "stm32f4xx_hal.h"

/**
 * @brief  Initializes the BMP180 handle using STM32 HAL I2C and Timer routines.
 * @param  handle: Pointer to the BMP180 driver handle.
 * @param  hi2c:   Pointer to the STM32 HAL I2C handle (e.g., &hi2c1).
 * @retval BMP180_OK on success, BMP180_ERROR on pointer null error.
 */
int8_t BMP180_Port_STM32_Init(BMP180_Handle_t *handle, I2C_HandleTypeDef *hi2c);

#endif /* BMP180_PORT_STM32_H */
