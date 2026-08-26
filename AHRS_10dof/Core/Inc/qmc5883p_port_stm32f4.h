/**
 ******************************************************************************
 * @file    qmc5883p_port_stm32f4.h
 * @author  abdurrahimozagac
 * @brief   STM32F4 Specific Port/Wrapper for QMC5883P Driver
 *
 * @details This file provides the function prototypes to link the generic
 *          QMC5883P driver with the STM32 HAL I2C driver.
 ******************************************************************************
 */

#ifndef QMC5883P_PORT_STM32_H
#define QMC5883P_PORT_STM32_H

#include "qmc5883p.h"
#include "stm32f4xx_hal.h"

/**
 * @brief  Initializes the QMC5883P handle using STM32 HAL I2C routines.
 * @param  handle: Pointer to the QMC5883P driver handle.
 * @param  hi2c:   Pointer to the STM32 HAL I2C handle (e.g., &hi2c1).
 * @retval QMC5883P_OK on success, QMC5883P_ERROR on pointer null error.
 */
int8_t QMC5883P_Port_STM32_Init(QMC5883P_Handle_t *handle, I2C_HandleTypeDef *hi2c);

#endif /* QMC5883P_PORT_STM32_H */
