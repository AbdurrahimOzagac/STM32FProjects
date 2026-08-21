
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

#ifndef QMC5883P_PORT_STM32_H
#define QMC5883P_PORT_STM32_H

#include "qmc5883p.h"
#include "stm32f4xx_hal.h"

int8_t QMC5883P_Port_STM32_Init(QMC5883P_Handle_t *handle, I2C_HandleTypeDef *hi2c);

#endif /* QMC5883P_PORT_STM32_H */
