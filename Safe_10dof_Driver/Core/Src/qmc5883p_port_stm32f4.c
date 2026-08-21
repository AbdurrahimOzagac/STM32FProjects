
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


#include "qmc5883p_port_stm32f4.h"
#include "i2c_safe.h"

static int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr,
		const uint8_t *data, uint16_t len, void *user_ctx) {
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) user_ctx;

	return I2C_Write_Safe(hi2c, (uint16_t) (dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT,
			(uint8_t *) data, len);

}

static int8_t stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len, void *user_ctx) {
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *) user_ctx;

	return I2C_Read_Safe(hi2c, (uint16_t) (dev_addr << 1), reg_addr, I2C_MEMADD_SIZE_8BIT,
			data, len);

}

int8_t QMC5883P_Port_STM32_Init(QMC5883P_Handle_t *handle, I2C_HandleTypeDef *hi2c) {
	if (handle == NULL || hi2c == NULL) {
		return -1;
	}

	return QMC5883P_I2C_Init(handle, stm32_i2c_write, stm32_i2c_read, (void *) hi2c);
}
