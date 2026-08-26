/**
 ******************************************************************************
 * @file    qmc5883p.h
 * @author  abdurrahimozagac
 * @brief   QMC5883P 3-Axis Magnetometer (Digital Compass) Driver
 *
 * @details Reads earth's magnetic field to determine Yaw (Heading) angle.
 *          Note: This is the QMC5883 (QST) variant, which uses different
 *          registers and I2C address (0x2C) compared to the older HMC5883L.
 *
 * @par Configuration Parameters (Ctrl_Reg1 - 0x0A):
 *      The MPU6050_Set_Mode() function configures the Output Data Rate (ODR),
 *      Over Sample Ratio (OSR), and Full Scale Range (RNG).
 *
 *      - Mode: Continuous (0x01) or Standby (0x00)
 *      - Output Data Rate (ODR): 10Hz, 50Hz, 100Hz, 200Hz
 *      - Full Scale Range (RNG): +-2 Gauss or +-8 Gauss
 *      - Over Sample Ratio (OSR): 512, 256, 128, 64 (Higher is less noisy)
 *
 *      @note The default mode (0xCF) sets the sensor to: Continuous Mode,
 *            200Hz ODR, 8G Range, and 64 OSR.
 *
 * @par Calibration Warning:
 *      Magnetometer data is heavily affected by "Hard Iron" (magnets/motors nearby)
 *      and "Soft Iron" (metals) distortions. The raw data (x_mag, y_mag, z_mag) MUST
 *      be calibrated (finding min/max offsets) before calculating the heading angle.
 *
 * @par Example Usage:
 *      @code
 *      QMC5883P_Handle_t qmc_handle;
 *      QMC5883P_MAGNETOMETER_t mag_data;
 *
 *      // 1. Initialization (Automatically sets 200Hz Continuous Mode)
 *      QMC5883P_Port_STM32_Init(&qmc_handle, &hi2c1);
 *
 *      // 2. Read Data (Inside main loop)
 *      if (QMC5883P_Get_Magnetic_Data(&qmc_handle, &mag_data) == QMC5883P_OK) {
 *          // Calibrate these raw values before calculating Heading = atan2(Y, X)
 *          // Example: mag_data.x_mag, mag_data.y_mag
 *      }
 *      @endcode
 ******************************************************************************
 */
#ifndef QMC5883P_H
#define QMC5883P_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


// --- SENSOR I2C ADDRESS AND WHOAMI ---
#define QMC5883P_ADDR                   0x2CU
#define QMC5883P_WHOAMI                 0x00

// --- REGISTER ADDRESSES ---
#define QMC5883P_REG_DATA_OUT_X_LSB     0x01
#define QMC5883P_REG_CTRL1              0x0A
#define QMC5883P_REG_SET_RESET          0x0B
#define QMC5883P_REG_PERIOD_CFG         0x29

// --- EXPECTED VALUES AND CONFIGURATIONS ---
#define QMC5883P_EXPECTED_WHOAMI        0x80
#define QMC5883P_CFG_PERIOD_DEF         0x06
#define QMC5883P_CFG_SET_RESET_DEF      0x08
#define QMC5883P_CFG_CTRL1_DEF          0xCF

// --- DATA LENGTHS ---
#define QMC5883P_CMD_LEN                1
#define QMC5883P_DATA_LEN               6

// --- STATUS AND ERROR CODES ---
#define QMC5883P_OK                     0
#define QMC5883P_ERROR                 -1
#define QMC5883P_ERR_WHOAMI            -2

/**
 * @brief Structure to hold 3-axis magnetometer raw data.
 */
typedef struct {
    int16_t x_mag; /*!< X-axis magnetic field raw data */
    int16_t y_mag; /*!< Y-axis magnetic field raw data */
    int16_t z_mag; /*!< Z-axis magnetic field raw data */
} QMC5883P_MAGNETOMETER_t;

/**
 * @brief Function pointers for I2C communication operations.
 */
typedef int8_t (*QMC5883P_I2C_Write_Fn)(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len, void *user_ctx);
typedef int8_t (*QMC5883P_I2C_Read_Fn)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len, void *user_ctx);

/**
 * @brief Main handle structure for the QMC5883P sensor.
 * @note  This structure must be properly initialized before using any API functions.
 */
typedef struct {
    QMC5883P_I2C_Write_Fn i2c_write; /*!< Pointer to the platform-specific I2C write function */
    QMC5883P_I2C_Read_Fn i2c_read;   /*!< Pointer to the platform-specific I2C read function */
    void *user_ctx;                  /*!< User context pointer (e.g., STM32 I2C handle) */
} QMC5883P_Handle_t;

/**
 * @brief  Initializes the QMC5883P sensor handle and verifies device identity.
 * @param  handle:   Pointer to the sensor handle structure.
 * @param  write_fn: Pointer to the I2C write implementation.
 * @param  read_fn:  Pointer to the I2C read implementation.
 * @param  user_ctx: Hardware-specific context (e.g., hi2c).
 * @retval QMC5883P_OK on success, QMC5883P_ERROR on pointer null error, QMC5883P_ERR_WHOAMI on chip ID mismatch.
 */
int8_t QMC5883P_I2C_Init(QMC5883P_Handle_t *handle, QMC5883P_I2C_Write_Fn write_fn, QMC5883P_I2C_Read_Fn read_fn, void *user_ctx);

/**
 * @brief  Sets specific configuration registers of the sensor.
 * @param  handle:     Pointer to the sensor handle structure.
 * @param  period_cfg: Period configuration value.
 * @param  set_reset:  Set/Reset register value.
 * @param  ctrl_reg1:  Control register 1 value (ODR, OSR, RNG, Mode).
 * @retval QMC5883P_OK on success, QMC5883P_ERROR on I2C failure.
 */
int8_t QMC5883P_Set_Mode(QMC5883P_Handle_t *handle, uint8_t period_cfg, uint8_t set_reset, uint8_t ctrl_reg1);

/**
 * @brief  Applies the default configuration (200Hz, +-30G, Continuous Mode).
 * @param  handle: Pointer to the sensor handle structure.
 * @retval QMC5883P_OK on success, QMC5883P_ERROR on I2C failure.
 */
int8_t QMC5883P_Set_Mode_Default(QMC5883P_Handle_t *handle);

/**
 * @brief  Reads the latest 3-axis magnetic field data from the sensor.
 * @param  handle:  Pointer to the sensor handle structure.
 * @param  out_mag: Pointer to the structure where the read data will be stored.
 * @retval QMC5883P_OK on success, QMC5883P_ERROR on I2C failure.
 */
int8_t QMC5883P_Get_Magnetic_Data(QMC5883P_Handle_t *handle, QMC5883P_MAGNETOMETER_t *out_mag);

#ifdef __cplusplus
}
#endif

#endif /* QMC5883P_H */
