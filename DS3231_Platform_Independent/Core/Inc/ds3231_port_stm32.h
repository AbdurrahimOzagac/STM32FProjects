/**
 ******************************************************************************
 * @file    ds3231_port_stm32.h
 * @brief   STM32 HAL port layer for the platform-independent DS3231 driver.
 *
 * @details This is the only header in the project that is allowed to
 *          depend on STM32 HAL. It wires a DS3231_Handle_t to a given
 *          I2C_HandleTypeDef by supplying HAL-backed I2C callbacks.
 *
 *          Porting to a different platform means writing an equivalent
 *          pair of files (e.g. ds3231_port_arduino.h/.c) — ds3231.h/.c
 *          remain untouched.
 ******************************************************************************
 */

#ifndef DS3231_PORT_STM32_H
#define DS3231_PORT_STM32_H

#include "ds3231.h"
#include "stm32f4xx_hal.h"

/**
 * @brief Bind a DS3231 handle to an STM32 HAL I2C peripheral.
 *
 * Registers STM32 HAL-based I2C read/write callbacks on @p handle and
 * stores @p hi2c as the opaque context passed back to those callbacks on
 * every transaction. Must be called once, after the I2C peripheral has
 * been initialized (e.g. after MX_I2C1_Init()), and before any
 * DS3231_Set_Time() / DS3231_Get_Time() call using this handle.
 *
 * @param[out] handle  Handle to configure for STM32 HAL use.
 * @param[in]  hi2c    Pointer to an already-initialized HAL I2C handle
 *                     (e.g. &hi2c1).
 *
 * @return 0 on success, -1 if @p handle or @p hi2c is NULL.
 */
int8_t DS3231_Port_STM32_Init(DS3231_Handle_t *handle, I2C_HandleTypeDef *hi2c);

#endif /* DS3231_PORT_STM32_H */
