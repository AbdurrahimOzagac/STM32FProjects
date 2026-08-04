/**
 ******************************************************************************
 * @file    ds3231.h
 * @brief   Platform-independent driver interface for the DS3231 RTC.
 *
 * @details This library has no dependency on any vendor HAL or MCU-specific
 *          header. All I2C transactions are delegated to user-supplied
 *          callback functions (see DS3231_I2C_Write_Fn / DS3231_I2C_Read_Fn),
 *          which are injected at runtime through DS3231_Init().
 *
 *          To port this driver to a new platform (STM32 HAL, Arduino Wire,
 *          ESP-IDF, Linux i2c-dev, ...), implement two small adapter
 *          functions matching the prototypes below and pass them to
 *          DS3231_Init(). This file and ds3231.c never need to change.
 *
 ******************************************************************************
 */

#ifndef DS3231_H
#define DS3231_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 7-bit I2C device address of the DS3231.
 * @note  This is the raw 7-bit address (0x68). Some platforms (e.g. STM32
 *        HAL) expect an 8-bit address (address << 1). Any such shifting
 *        is the responsibility of the platform-specific port layer, not
 *        of this driver.
 */
#define DS3231_I2C_ADDRESS   0x68U

/**
 * @brief Human-readable representation of the DS3231's current date/time.
 *
 * All fields are stored as plain decimal values (already converted from
 * the chip's internal BCD encoding).
 */
typedef struct {
	uint8_t sec;         /**< Seconds,       0-59 */
	uint8_t min;         /**< Minutes,       0-59 */
	uint8_t hour;         /**< Hour,          0-23 (24-hour mode) */
	uint8_t dayOfWeek;    /**< Day of week,   1-7 (chip-defined convention) */
	uint8_t dayOfMonth;   /**< Day of month,  1-31 */
	uint8_t month;        /**< Month,         1-12 */
	uint8_t year;         /**< Year,          0-99 (offset from a fixed century) */
} DS3231_Time_t;

/**
 * @brief Platform-supplied I2C write callback.
 *
 * Implementations must write @p len bytes from @p data into the device's
 * registers, starting at @p reg_addr, using whatever I2C stack the target
 * platform provides.
 *
 * @param[in] dev_addr  7-bit I2C address of the target device
 *                      (always DS3231_I2C_ADDRESS for this driver).
 * @param[in] reg_addr  Register address to start writing at.
 * @param[in] data      Buffer holding the bytes to write.
 * @param[in] len       Number of bytes to write.
 * @param[in] user_ctx  Opaque platform context, forwarded unmodified from
 *                       DS3231_Handle_t::user_ctx (e.g. an I2C bus handle).
 *
 * @return 0 on success, non-zero on failure.
 */
typedef int8_t (*DS3231_I2C_Write_Fn)(uint8_t dev_addr, uint8_t reg_addr,
		const uint8_t *data, uint16_t len, void *user_ctx);

/**
 * @brief Platform-supplied I2C read callback.
 *
 * Implementations must read @p len bytes starting at @p reg_addr into
 * @p data, using whatever I2C stack the target platform provides.
 *
 * @param[in]  dev_addr  7-bit I2C address of the target device
 *                       (always DS3231_I2C_ADDRESS for this driver).
 * @param[in]  reg_addr  Register address to start reading from.
 * @param[out] data      Buffer to be filled with the bytes read.
 * @param[in]  len       Number of bytes to read.
 * @param[in]  user_ctx  Opaque platform context, forwarded unmodified from
 *                       DS3231_Handle_t::user_ctx (e.g. an I2C bus handle).
 *
 * @return 0 on success, non-zero on failure.
 */
typedef int8_t (*DS3231_I2C_Read_Fn)(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len, void *user_ctx);

/**
 * @brief Instance handle bundling everything needed to talk to one
 *        DS3231 device.
 *
 * Multiple handles may coexist (e.g. two DS3231 chips on two different
 * I2C buses); each handle carries its own callbacks and context, so the
 * driver never mixes them up.
 */
typedef struct {
	DS3231_I2C_Write_Fn i2c_write; /**< Registered I2C write callback. */
	DS3231_I2C_Read_Fn i2c_read;   /**< Registered I2C read callback. */
	void *user_ctx;                /**< Opaque platform-specific context
	                                     (e.g. I2C_HandleTypeDef*). */
} DS3231_Handle_t;

/**
 * @brief Initialize a DS3231 handle with platform-specific I2C callbacks.
 *
 * Must be called once before any other DS3231_* function is used with
 * this handle. Typically not called directly by application code; a
 * platform port layer (e.g. DS3231_Port_STM32_Init()) usually wraps it.
 *
 * @param[out] handle    Handle to initialize.
 * @param[in]  write_fn  Callback implementing I2C register writes.
 * @param[in]  read_fn   Callback implementing I2C register reads.
 * @param[in]  user_ctx  Opaque context forwarded to write_fn/read_fn on
 *                       every call (e.g. a bus handle).
 */
void DS3231_Init(DS3231_Handle_t *handle, DS3231_I2C_Write_Fn write_fn,
		DS3231_I2C_Read_Fn read_fn, void *user_ctx);

/**
 * @brief Write a new date/time to the DS3231.
 *
 * All fields are converted from decimal to BCD internally before being
 * transmitted, matching the chip's native register format.
 *
 * @param[in] handle      Initialized DS3231 handle.
 * @param[in] sec         Seconds,      0-59.
 * @param[in] min         Minutes,      0-59.
 * @param[in] hour        Hour,         0-23.
 * @param[in] dayOfWeek   Day of week,  1-7.
 * @param[in] dayOfMonth  Day of month, 1-31.
 * @param[in] month       Month,        1-12.
 * @param[in] year        Year,         0-99.
 *
 * @return 0 on success, non-zero on I2C failure (error code propagated
 *         from the underlying port layer).
 */
int8_t DS3231_Set_Time(DS3231_Handle_t *handle, uint8_t sec, uint8_t min,
		uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month,
		uint8_t year);

/**
 * @brief Read the current date/time from the DS3231.
 *
 * On success, @p time is populated with decimal values converted from
 * the chip's BCD registers. On failure, @p time is left untouched so
 * callers never observe partially-updated or garbage data.
 *
 * @param[in]  handle  Initialized DS3231 handle.
 * @param[out] time    Destination struct to receive the current time.
 *
 * @return 0 on success, non-zero on I2C failure (error code propagated
 *         from the underlying port layer).
 */
int8_t DS3231_Get_Time(DS3231_Handle_t *handle, DS3231_Time_t *time);

#ifdef __cplusplus
}
#endif

#endif /* DS3231_H */
