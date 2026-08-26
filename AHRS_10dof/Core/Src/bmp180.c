/**
 ******************************************************************************
 * @file    bmp180.c
 * @author  abdurrahimozagac
 * @brief   BMP180 Pressure and Temperature Sensor Driver Implementation
 ******************************************************************************
 */

#include "bmp180.h"
#include <stddef.h>
#include <math.h>

/* Lookup table for oversampling delays based on OSS mode */
static const uint8_t bmp_oss_delays[4] = { 5, 8, 14, 26 };

static int8_t BMP180_Read_Calibration(BMP180_Handle_t *handle);
static void BMP180_Get_Temperature_Celsius(uint16_t ut, BMP180_Calib_t *calib, long *out_b5_ref, float *out_tempc);
static void BMP180_Get_Pressure_Pa(uint32_t upressure, BMP180_Calib_t *calib, long *b5_ref, OSS oss, float *out_pressure_pa);

int8_t BMP180_I2C_Init(BMP180_Handle_t *handle, BMP180_I2C_Write_Fn write_fn,
        BMP180_I2C_Read_Fn read_fn, BMP180_GetTick_Fn tick_fn, void *ctx) {

    if (handle == NULL || write_fn == NULL || read_fn == NULL || tick_fn == NULL) {
        return BMP180_ERROR;
    }

    handle->i2c_write = write_fn;
    handle->i2c_read  = read_fn;
    handle->get_tick  = tick_fn;
    handle->user_ctx  = ctx;

    handle->state      = BMP180_STATE_WAIT_TEMP;
    handle->data_valid = 0;
    handle->sea_level_pa = BMP180_DEFAULT_SEA_LEVEL_PA;

    // --- WHO AM I (CHIP ID) CHECK ---
    uint8_t chip_id = 0;
    if (handle->i2c_read(BMP180_ADDR, BMP180_CHIP_ID_REG, &chip_id, BMP180_CMD_LEN, handle->user_ctx) != BMP180_OK) {
        return BMP180_ERROR; // No response from I2C bus
    }

    if (chip_id != BMP180_EXPECTED_CHIP_ID) {
        return BMP180_ERROR; // Device is not BMP180 or read error occurred
    }
    // ------------------------------------

    int8_t status = BMP180_Read_Calibration(handle);
    if (status != BMP180_OK) {
        return status;
    }

    uint8_t cmd = BMP180_COMMAND_TEMP;
    if (handle->i2c_write(BMP180_ADDR, BMP180_CONTROL_REG, &cmd, BMP180_CMD_LEN, ctx) != BMP180_OK) {
        return BMP180_ERROR;
    }

    handle->state = BMP180_STATE_WAIT_TEMP;
    handle->timer = handle->get_tick();

    return BMP180_OK;
}

int8_t BMP180_Poll(BMP180_Handle_t *handle) {

    uint8_t command = 0;

    switch (handle->state) {
        case BMP180_STATE_WAIT_TEMP: {

            if (handle->get_tick() - handle->timer < BMP180_TEMP_MEAS_DELAY_MS) {
                return BMP180_BUSY;
            }

            uint8_t temp_raw_data[BMP180_TEMP_DATA_LEN];

            // Read raw temperature data
            if (handle->i2c_read(BMP180_ADDR, BMP180_READ_REG, temp_raw_data,
                                 BMP180_TEMP_DATA_LEN, handle->user_ctx) != BMP180_OK) {
                return BMP180_ERROR;
            }

            handle->data.utemp = (temp_raw_data[0] << 8) | temp_raw_data[1];

            // Send command to initiate pressure measurement
            command = BMP180_COMMAND_PRESS + (handle->oss << 6);
            if (handle->i2c_write(BMP180_ADDR, BMP180_CONTROL_REG, &command,
                                  BMP180_CMD_LEN, handle->user_ctx) != BMP180_OK) {
                return BMP180_ERROR;
            }

            handle->state = BMP180_STATE_WAIT_PRESS;
            handle->timer = handle->get_tick();

            return BMP180_OK;
        }

        case BMP180_STATE_WAIT_PRESS: {

            if (handle->get_tick() - handle->timer < bmp_oss_delays[handle->oss]) {
                return BMP180_BUSY;
            }

            uint8_t pressure_raw_data[BMP180_PRESS_DATA_LEN];

            // Read raw pressure data
            if (handle->i2c_read(BMP180_ADDR, BMP180_READ_REG, pressure_raw_data,
                                 BMP180_PRESS_DATA_LEN, handle->user_ctx) != BMP180_OK) {
                return BMP180_ERROR;
            }

            handle->data.upressure = (((uint32_t) pressure_raw_data[0] << 16)
                    | ((uint32_t) pressure_raw_data[1] << 8)
                    | ((uint32_t) pressure_raw_data[2])) >> (8 - handle->oss);

            // Convert raw data to meaningful physical values using calibration math
            BMP180_Get_Temperature_Celsius(handle->data.utemp, &handle->calib,
                    &handle->_b5, &handle->data.temperature_c);

            BMP180_Get_Pressure_Pa(handle->data.upressure, &handle->calib,
                    &handle->_b5, handle->oss, &handle->data.pressure_pa);

            // Calculate altitude based on current sea-level reference
            handle->data.altitude_m = 44330.0f * (1.0f - powf(handle->data.pressure_pa / handle->sea_level_pa, 0.1902949f));

            // Send temperature command to prepare for the next cycle
            command = BMP180_COMMAND_TEMP;
            if (handle->i2c_write(BMP180_ADDR, BMP180_CONTROL_REG, &command,
                                  BMP180_CMD_LEN, handle->user_ctx) != BMP180_OK) {
                return BMP180_ERROR;
            }

            handle->state = BMP180_STATE_WAIT_TEMP;
            handle->timer = handle->get_tick();

            // Update the latest atomic data mailbox for thread-safe reading
            handle->latest_atomic_data = handle->data;
            handle->data_valid = 1;

            return BMP180_OK;
        }
    }

    // If state machine exits unexpectedly
    return BMP180_ERROR;
}

static int8_t BMP180_Read_Calibration(BMP180_Handle_t *handle) {

    uint8_t buffer[BMP180_CALIB_DATA_LEN];

    // Read all 22 calibration bytes in a single burst
    if (handle->i2c_read(BMP180_ADDR, BMP180_CALIB_START_REG, buffer,
                         BMP180_CALIB_DATA_LEN, handle->user_ctx) != BMP180_OK) {
        return BMP180_ERROR;
    }

    handle->calib.AC1 = (buffer[0] << 8)  | buffer[1];
    handle->calib.AC2 = (buffer[2] << 8)  | buffer[3];
    handle->calib.AC3 = (buffer[4] << 8)  | buffer[5];
    handle->calib.AC4 = (buffer[6] << 8)  | buffer[7];
    handle->calib.AC5 = (buffer[8] << 8)  | buffer[9];
    handle->calib.AC6 = (buffer[10] << 8) | buffer[11];
    handle->calib.B1  = (buffer[12] << 8) | buffer[13];
    handle->calib.B2  = (buffer[14] << 8) | buffer[15];
    handle->calib.MB  = (buffer[16] << 8) | buffer[17];
    handle->calib.MC  = (buffer[18] << 8) | buffer[19];
    handle->calib.MD  = (buffer[20] << 8) | buffer[21];

    return BMP180_OK;
}

static void BMP180_Get_Temperature_Celsius(uint16_t ut, BMP180_Calib_t *calib,
        long *out_b5_ref, float *out_tempc) {
    long x1, x2;
    long temp_tenth = 0;

    x1 = (((long) ut - (long) calib->AC6) * (long) calib->AC5) >> 15;
    x2 = ((long) calib->MC << 11) / (x1 + (long) calib->MD);

    // Variable B5 is required for subsequent pressure calculation
    *out_b5_ref = x1 + x2;

    temp_tenth = (*out_b5_ref + 8) >> 4;
    *out_tempc = temp_tenth / 10.0f;
}

static void BMP180_Get_Pressure_Pa(uint32_t upressure, BMP180_Calib_t *calib,
        long *b5_ref, OSS oss, float *out_pressure_pa) {
    long x1, x2, x3, b3, b6;
    unsigned long b4, b7, p;

    b6 = *b5_ref - 4000;

    x1 = ((long) calib->B2 * ((b6 * b6) >> 12)) >> 11;
    x2 = ((long) calib->AC2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((long) calib->AC1) * 4 + x3) << oss) + 2) >> 2;

    x1 = ((long) calib->AC3 * b6) >> 13;
    x2 = ((long) calib->B1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = ((long) calib->AC4 * (unsigned long) (x3 + 32768)) >> 15;

    b7 = ((unsigned long) upressure - b3) * (50000 >> oss);

    if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
    } else {
        p = (b7 / b4) << 1;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    *out_pressure_pa = (float) p;
}

int8_t BMP180_Get_Data(BMP180_Handle_t *handle, BMP180_Data_t *out_data) {

    if (handle == NULL || out_data == NULL || !handle->data_valid) {
        return BMP180_ERROR;
    }

    *out_data = handle->latest_atomic_data;

    return BMP180_OK;
}

void BMP180_Set_Sea_Level_Pressure(BMP180_Handle_t *handle, float pressure_pa) {
    if (handle != NULL && pressure_pa > BMP180_MIN_VALID_PRESSURE_PA) {
        handle->sea_level_pa = pressure_pa;
    }
}
