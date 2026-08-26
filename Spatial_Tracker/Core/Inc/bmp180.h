/**
 ******************************************************************************
 * @file    bmp180.h
 * @author  abdurrahimozagac
 * @brief   BMP180 Pressure, Temperature and Altitude Sensor Driver
 *
 * @details This driver provides a completely non-blocking state-machine based
 *          interface for the BMP180 sensor. It calculates temperature in Celsius,
 *          pressure in Pascals, and absolute altitude in meters.
 *
 * @par Configuration Parameters (OSS - Oversampling Setting):
 *      The BMP180 has 4 accuracy modes that trade off resolution for speed and power:
 *      - OSS0 (Ultra Low Power): 1 sample, ~4.5ms delay. Good for power saving.
 *      - OSS1 (Standard)       : 2 samples, ~7.5ms delay.
 *      - OSS2 (High Resolution): 4 samples, ~13.5ms delay.
 *      - OSS3 (Ultra High Res) : 8 samples, ~25.5ms delay. Best for altitude.
 *      @note Set handle->oss = OSS3 before calling BMP180_Poll() for best drone/AHRS results.
 *
 * @par Altitude and Sea-Level Pressure:
 *      Altitude is calculated using the barometric formula. Since weather changes
 *      the baseline pressure, you MUST update the sea-level reference for accurate
 *      absolute altitude:
 *      @code BMP180_Set_Sea_Level_Pressure(&handle, 101325.0f); @endcode
 *
 * @par Example Usage:
 *      @code
 *      BMP180_Handle_t bmp_handle;
 *      BMP180_Data_t bmp_data;
 *
 *      // 1. Initialization
 *      if (BMP180_Port_STM32_Init(&bmp_handle, &hi2c1) == BMP180_OK) {
 *          bmp_handle.oss = OSS3; // Set highest resolution
 *      }
 *
 *      // 2. Non-Blocking Loop (Inside main while loop)
 *      if (BMP180_Poll(&bmp_handle) == BMP180_OK) {
 *          // New data is ready!
 *          BMP180_Get_Data(&bmp_handle, &bmp_data);
 *          // Use bmp_data.temperature_c, bmp_data.pressure_pa, bmp_data.altitude_m
 *      }
 *      @endcode
 ******************************************************************************
 */

#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- SENSOR I2C ADDRESS AND COMMANDS ---
#define BMP180_ADDR                     0x77
#define BMP180_COMMAND_TEMP             0x2E
#define BMP180_COMMAND_PRESS            0x34

// --- CHIP ID AND REGISTERS ---
#define BMP180_CHIP_ID_REG              0xD0
#define BMP180_EXPECTED_CHIP_ID         0x55

// --- REGISTER ADDRESSES AND LENGTHS ---
#define BMP180_CONTROL_REG              0xF4
#define BMP180_READ_REG                 0xF6
#define BMP180_CALIB_START_REG          0xAA

#define BMP180_CALIB_DATA_LEN           22
#define BMP180_TEMP_DATA_LEN            2
#define BMP180_PRESS_DATA_LEN           3
#define BMP180_CMD_LEN                  1

// --- DELAY TIMES (ms) ---
#define BMP180_TEMP_MEAS_DELAY_MS       5

// --- STATUS AND ERROR CODES ---
#define BMP180_OK                       0
#define BMP180_ERROR                    -1
#define BMP180_BUSY                     -2

// --- REFERENCE VALUES ---
#define BMP180_DEFAULT_SEA_LEVEL_PA     101325.0f
#define BMP180_MIN_VALID_PRESSURE_PA    1000.0f

/**
 * @brief Structure to hold BMP180 factory calibration coefficients.
 */
typedef struct {
    int16_t AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t B1, B2, MB, MC, MD;
} BMP180_Calib_t;

/**
 * @brief Structure to hold calculated and raw sensor data.
 */
typedef struct {
    float temperature_c; /*!< Calculated temperature in Celsius */
    float pressure_pa;   /*!< Calculated pressure in Pascals */
    float altitude_m;    /*!< Calculated altitude in meters */
    uint16_t utemp;      /*!< Uncompensated raw temperature */
    uint32_t upressure;  /*!< Uncompensated raw pressure */
} BMP180_Data_t;

/**
 * @brief State machine enumeration for non-blocking sensor polling.
 */
typedef enum {
    BMP180_STATE_WAIT_TEMP,
    BMP180_STATE_WAIT_PRESS
} BMP180_State_t;

/**
 * @brief Oversampling settings for pressure measurement resolution.
 */
typedef enum {
    OSS0, /*!< Ultra low power (1 sample, 4.5ms) */
    OSS1, /*!< Standard (2 samples, 7.5ms) */
    OSS2, /*!< High resolution (4 samples, 13.5ms) */
    OSS3  /*!< Ultra high resolution (8 samples, 25.5ms) */
} OSS;

/**
 * @brief Function pointers for generic I2C communication and timing.
 */
typedef int8_t (*BMP180_I2C_Write_Fn)(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len, void *user_ctx);
typedef int8_t (*BMP180_I2C_Read_Fn)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len, void *user_ctx);
typedef uint32_t (*BMP180_GetTick_Fn)(void);

/**
 * @brief Main handle structure for the BMP180 sensor.
 */
typedef struct {
    BMP180_I2C_Write_Fn i2c_write; /*!< Pointer to the platform-specific I2C write function */
    BMP180_I2C_Read_Fn i2c_read;   /*!< Pointer to the platform-specific I2C read function */
    BMP180_GetTick_Fn get_tick;    /*!< Pointer to the platform-specific tick function (e.g., HAL_GetTick) */
    void *user_ctx;                /*!< User context pointer (e.g., STM32 I2C handle) */

    BMP180_Calib_t calib;          /*!< Calibration coefficients */
    BMP180_State_t state;          /*!< Current state of the polling state machine */
    uint32_t timer;                /*!< Timer variable for non-blocking delays */
    long _b5;                      /*!< Intermediate temperature variable needed for pressure calculation */
    OSS oss;                       /*!< Current oversampling setting */
    float sea_level_pa;            /*!< Sea-level pressure reference for altitude calculation */
    int8_t data_valid;             /*!< Flag indicating if valid data is available */

    BMP180_Data_t data;               /*!< Internal working data buffer */
    BMP180_Data_t latest_atomic_data; /*!< Latest complete data available for user retrieval */

} BMP180_Handle_t;

/**
 * @brief  Initializes the BMP180 sensor, verifies Chip ID, and reads calibration data.
 * @param  handle:   Pointer to the sensor handle structure.
 * @param  write_fn: Pointer to the I2C write implementation.
 * @param  read_fn:  Pointer to the I2C read implementation.
 * @param  tick_fn:  Pointer to the system tick implementation.
 * @param  ctx:      Hardware-specific context (e.g., hi2c).
 * @retval BMP180_OK on success, BMP180_ERROR on failure.
 */
int8_t BMP180_I2C_Init(BMP180_Handle_t *handle, BMP180_I2C_Write_Fn write_fn, BMP180_I2C_Read_Fn read_fn, BMP180_GetTick_Fn tick_fn, void *ctx);

/**
 * @brief  State machine function to poll the sensor non-blockingly. Must be called periodically.
 * @param  handle: Pointer to the sensor handle structure.
 * @retval BMP180_OK if a full cycle is complete, BMP180_BUSY if waiting, BMP180_ERROR on failure.
 */
int8_t BMP180_Poll(BMP180_Handle_t *handle);

/**
 * @brief  Retrieves the latest safely constructed data from the sensor handle.
 * @param  h:   Pointer to the sensor handle structure.
 * @param  out: Pointer to the user-provided data structure to store the results.
 * @retval BMP180_OK on success, BMP180_ERROR if no valid data is available yet.
 */
int8_t BMP180_Get_Data(BMP180_Handle_t *h, BMP180_Data_t *out);

/**
 * @brief  Sets a new sea-level pressure reference to calibrate the altitude output.
 * @param  handle:      Pointer to the sensor handle structure.
 * @param  pressure_pa: Current sea-level pressure in Pascals.
 */
void BMP180_Set_Sea_Level_Pressure(BMP180_Handle_t *handle, float pressure_pa);

#ifdef __cplusplus
}
#endif

#endif /* BMP180_H */
