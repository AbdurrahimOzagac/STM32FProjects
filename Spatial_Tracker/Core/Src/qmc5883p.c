/**
 ******************************************************************************
 * @file    qmc5883p.c
 * @author  abdurrahimozagac
 * @brief   QMC5883P Magnetometer Driver Implementation
 ******************************************************************************
 */

#include "qmc5883p.h"

int8_t QMC5883P_I2C_Init(QMC5883P_Handle_t *handle,
        QMC5883P_I2C_Write_Fn write_fn, QMC5883P_I2C_Read_Fn read_fn,
        void *user_ctx) {

    // Assign communication functions and user context to the handle
    handle->i2c_write = write_fn;
    handle->i2c_read = read_fn;
    handle->user_ctx = user_ctx;

    uint8_t chip_id = 0;

    // Check device ID to ensure QMC5883P is correctly connected
    int8_t status = handle->i2c_read(QMC5883P_ADDR, QMC5883P_WHOAMI, &chip_id, QMC5883P_CMD_LEN,
            handle->user_ctx);

    if (status != QMC5883P_OK) return status;

    if(chip_id != QMC5883P_EXPECTED_WHOAMI) return QMC5883P_ERR_WHOAMI;

    // Proceed to set default operational mode if identification is successful
    return QMC5883P_Set_Mode_Default(handle);
}

int8_t QMC5883P_Set_Mode(QMC5883P_Handle_t *handle, uint8_t period_cfg,
        uint8_t set_reset, uint8_t ctrl_reg1) {

    int8_t status = QMC5883P_OK;

    // Write Period Configuration
    status = handle->i2c_write(QMC5883P_ADDR, QMC5883P_REG_PERIOD_CFG, &period_cfg, QMC5883P_CMD_LEN,
            handle->user_ctx);
    if (status != QMC5883P_OK) return status;

    // Write Set/Reset Configuration
    status = handle->i2c_write(QMC5883P_ADDR, QMC5883P_REG_SET_RESET, &set_reset, QMC5883P_CMD_LEN,
            handle->user_ctx);
    if (status != QMC5883P_OK) return status;

    // Write Control Register 1 Configuration
    status = handle->i2c_write(QMC5883P_ADDR, QMC5883P_REG_CTRL1, &ctrl_reg1, QMC5883P_CMD_LEN,
            handle->user_ctx);
    if (status != QMC5883P_OK) return status;

    return QMC5883P_OK;
}

int8_t QMC5883P_Set_Mode_Default(QMC5883P_Handle_t *handle) {
    // Configures the sensor with predefined default macros
    return QMC5883P_Set_Mode(handle, QMC5883P_CFG_PERIOD_DEF, QMC5883P_CFG_SET_RESET_DEF, QMC5883P_CFG_CTRL1_DEF);
}

int8_t QMC5883P_Get_Magnetic_Data(QMC5883P_Handle_t *handle,
        QMC5883P_MAGNETOMETER_t *out_mag) {

    uint8_t status = QMC5883P_OK;
    uint8_t buffer[QMC5883P_DATA_LEN] = { 0 };

    // Read 6 bytes of continuous magnetic data starting from the X LSB register
    status = handle->i2c_read(QMC5883P_ADDR, QMC5883P_REG_DATA_OUT_X_LSB, buffer, QMC5883P_DATA_LEN, handle->user_ctx);

    if (status != QMC5883P_OK) return status;

    // Combine High and Low bytes (Little-Endian Format)
    out_mag->x_mag = (int16_t) ((buffer[1] << 8) | buffer[0]);
    out_mag->y_mag = (int16_t) ((buffer[3] << 8) | buffer[2]);
    out_mag->z_mag = (int16_t) ((buffer[5] << 8) | buffer[4]);

    return QMC5883P_OK;
}
