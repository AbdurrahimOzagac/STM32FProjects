/**
 ******************************************************************************
 * @file    bmp180_port_stm32f4.c
 * @author  abdurrahimozagac
 * @brief   STM32F4 Specific Port/Wrapper Implementation for BMP180 Driver
 ******************************************************************************
 */

#include "bmp180_port_stm32f4.h"
#include "i2c_safe.h"

/**
 * @brief  Wrapper for STM32 HAL I2C write function.
 */
static int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr,
        const uint8_t *data, uint16_t len, void *user_ctx) {
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) user_ctx;

    // Shift device address left by 1 bit as expected by STM32 HAL
    return I2C_Write_Safe(hi2c, (uint16_t) (dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT,
            (uint8_t *) data, len);
}

/**
 * @brief  Wrapper for STM32 HAL I2C read function.
 */
static int8_t stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr,
        uint8_t *data, uint16_t len, void *user_ctx) {
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) user_ctx;

    // Shift device address left by 1 bit as expected by STM32 HAL
    return I2C_Read_Safe(hi2c, (uint16_t) (dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT,
            data, len);
}

/**
 * @brief  Wrapper for STM32 HAL system tick function.
 */
static uint32_t stm32_gettick(void) {
    return HAL_GetTick();
}

int8_t BMP180_Port_STM32_Init(BMP180_Handle_t *handle, I2C_HandleTypeDef *hi2c) {
    if (handle == NULL || hi2c == NULL) {
        return BMP180_ERROR;
    }

    // Pass the STM32 specific functions and I2C handle to the generic driver
    return BMP180_I2C_Init(handle, stm32_i2c_write, stm32_i2c_read, stm32_gettick, (void *) hi2c);
}
