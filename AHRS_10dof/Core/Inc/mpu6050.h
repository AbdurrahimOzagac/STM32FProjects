/**
 ******************************************************************************
 * @file    mpu6050.h
 * @author  abdurrahimozagac
 * @brief   MPU6050 6-Axis Inertial Measurement Unit (IMU) Driver
 *
 * @details Provides a lightweight, burst-read optimized driver for the MPU6050
 *          accelerometer and gyroscope. Essential for AHRS and Sensor Fusion.
 *
 * @par Configuration Parameters:
 *      When calling MPU6050_Set_Mode(), you must configure the physical ranges.
 *
 *      1. Accelerometer Range (accel_mode):
 *         - 0x00: +- 2g  (Highest sensitivity, best for slow tilts)
 *         - 0x08: +- 4g
 *         - 0x10: +- 8g  (Recommended for drones)
 *         - 0x18: +- 16g (Best for high impact/acrobatics)
 *
 *      2. Gyroscope Range (gyro_mode):
 *         - 0x00: 250 dps (degrees per second)
 *         - 0x08: 500 dps
 *         - 0x10: 1000 dps
 *         - 0x18: 2000 dps (Recommended for fast-spinning drones)
 *
 *      3. Digital Low Pass Filter (dlpf_mode):
 *         - 0x00: 260Hz (Fastest, very noisy)
 *         - 0x03: 42Hz  (Balanced, removes motor vibrations)
 *         - 0x06: 5Hz   (Smoothest, but has delay/lag)
 *
 * @par Example Usage:
 *      @code
 *      MPU6050_Handle_t mpu_handle;
 *      MPU6050_Inertial_t mpu_data;
 *
 *      // 1. Initialization (Automatically sets defaults: +-2g, 250dps, 42Hz DLPF)
 *      MPU6050_Port_STM32_Init(&mpu_handle, &hi2c1);
 *
 *      // Optional: Override defaults for a drone (8g, 1000dps, 42Hz DLPF)
 *      // MPU6050_Set_Mode(&mpu_handle, 0x00, 0x10, 0x10, 0x03);
 *
 *      // 2. Read Data (Inside main loop or timer interrupt)
 *      if (MPU6050_Get_Inertial_Data(&mpu_handle, &mpu_data) == MPU6050_OK) {
 *          // Raw values retrieved! Pass them to a Kalman or Mahony filter.
 *          // Example: mpu_data.accel.accel_x, mpu_data.gyro.gyro_z
 *      }
 *      @endcode
 ******************************************************************************
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- SENSOR I2C ADDRESS AND WHOAMI ---
#define MPU6050_ADDR                0x68
#define MPU6050_WHOAMI              0x75

// --- REGISTER ADDRESSES ---
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_PWR_MGMT_1      0x6B

// --- EXPECTED VALUES AND SETTINGS ---
#define MPU6050_EXPECTED_WHOAMI     0x68
#define MPU6050_WAKE_UP_VAL         0x00
#define MPU6050_ACCEL_2G_VAL        0x00
#define MPU6050_GYRO_250DPS_VAL     0x00
#define MPU6050_DLPF_42HZ_VAL       0x03

// --- DATA LENGTHS ---
#define MPU6050_CMD_LEN             1  /*!< Single byte command length */
#define MPU6050_DATA_LEN_SINGLE     6  /*!< 6 bytes for Accel or Gyro only (X,Y,Z * 2 bytes) */
#define MPU6050_DATA_LEN_ALL        14 /*!< 14 bytes for Accel (6) + Temp (2) + Gyro (6) */

// --- STATUS AND ERROR CODES ---
#define MPU6050_OK                  0
#define MPU6050_ERROR              -1
#define MPU6050_ERR_WHOAMI         -2

/**
 * @brief Structure to hold 3-axis raw acceleration data.
 */
typedef struct {
    int16_t accel_x; /*!< X-axis acceleration raw data */
    int16_t accel_y; /*!< Y-axis acceleration raw data */
    int16_t accel_z; /*!< Z-axis acceleration raw data */
} MPU6050_Acceleration_t;

/**
 * @brief Structure to hold 3-axis raw gyroscope data.
 */
typedef struct {
    int16_t gyro_x; /*!< X-axis gyroscope raw data */
    int16_t gyro_y; /*!< Y-axis gyroscope raw data */
    int16_t gyro_z; /*!< Z-axis gyroscope raw data */
} Gyroscope_t;

/**
 * @brief Structure to hold combined inertial data (Acceleration + Gyroscope).
 */
typedef struct {
    MPU6050_Acceleration_t accel; /*!< Acceleration data structure */
    Gyroscope_t gyro;             /*!< Gyroscope data structure */
} MPU6050_Inertial_t;

/**
 * @brief Function pointers for generic I2C communication.
 */
typedef int8_t (*MPU6050_I2C_Write_Fn)(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len, void *user_ctx);
typedef int8_t (*MPU6050_I2C_Read_Fn)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len, void *user_ctx);

/**
 * @brief Main handle structure for the MPU6050 sensor.
 */
typedef struct {
    MPU6050_I2C_Write_Fn i2c_write; /*!< Pointer to the platform-specific I2C write function */
    MPU6050_I2C_Read_Fn i2c_read;   /*!< Pointer to the platform-specific I2C read function */
    void *user_ctx;                 /*!< User context pointer (e.g., STM32 I2C handle) */
} MPU6050_Handle_t;

/**
 * @brief  Initializes the MPU6050 sensor handle and verifies the device ID.
 * @param  handle:   Pointer to the MPU6050 handle structure.
 * @param  write_fn: Pointer to the I2C write implementation.
 * @param  read_fn:  Pointer to the I2C read implementation.
 * @param  user_ctx: Hardware-specific context (e.g., hi2c).
 * @retval MPU6050_OK on success, MPU6050_ERROR on failure, MPU6050_ERR_WHOAMI on ID mismatch.
 */
int8_t MPU6050_I2C_Init(MPU6050_Handle_t *handle, MPU6050_I2C_Write_Fn write_fn, MPU6050_I2C_Read_Fn read_fn, void *user_ctx);

/**
 * @brief  Configures the MPU6050 wake state, scale ranges, and digital low pass filter.
 * @param  handle:     Pointer to the MPU6050 handle structure.
 * @param  wake:       Power management configuration (wake up).
 * @param  accel_mode: Accelerometer full scale range configuration.
 * @param  gyro_mode:  Gyroscope full scale range configuration.
 * @param  dlpf_mode:  Digital Low Pass Filter configuration.
 * @retval MPU6050_OK on success, MPU6050_ERROR on I2C failure.
 */
int8_t MPU6050_Set_Mode(MPU6050_Handle_t *handle, uint8_t wake, uint8_t accel_mode, uint8_t gyro_mode, uint8_t dlpf_mode);

/**
 * @brief  Applies the default configuration (+-2g, +-250dps, 42Hz DLPF).
 * @param  handle: Pointer to the MPU6050 handle structure.
 * @retval MPU6050_OK on success, MPU6050_ERROR on I2C failure.
 */
int8_t MPU6050_Set_Mode_Default(MPU6050_Handle_t *handle);

/**
 * @brief  Reads only the 3-axis acceleration data.
 * @param  handle:  Pointer to the MPU6050 handle structure.
 * @param  out_acc: Pointer to the structure where acceleration data will be stored.
 * @retval MPU6050_OK on success, MPU6050_ERROR on I2C failure.
 */
int8_t MPU6050_Get_Acceleration_Data(MPU6050_Handle_t *handle, MPU6050_Acceleration_t *out_acc);

/**
 * @brief  Reads only the 3-axis gyroscope data.
 * @param  handle:   Pointer to the MPU6050 handle structure.
 * @param  out_gyro: Pointer to the structure where gyroscope data will be stored.
 * @retval MPU6050_OK on success, MPU6050_ERROR on I2C failure.
 */
int8_t MPU6050_Get_Gyroscope_Data(MPU6050_Handle_t *handle, Gyroscope_t *out_gyro);

/**
 * @brief  Reads both acceleration and gyroscope data in a single burst read.
 * @param  handle:      Pointer to the MPU6050 handle structure.
 * @param  out_inertia: Pointer to the combined inertial data structure.
 * @retval MPU6050_OK on success, MPU6050_ERROR on I2C failure.
 */
int8_t MPU6050_Get_Inertial_Data(MPU6050_Handle_t *handle, MPU6050_Inertial_t *out_inertia);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
