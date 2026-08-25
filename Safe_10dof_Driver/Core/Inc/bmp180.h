/**
 ******************************************************************************
 * @file    bmp180.h
 * @brief   BMP180 Pressure and Temperature Sensor Driver
 ******************************************************************************
 */


#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
// --- SENSÖR I2C ADRESİ VE KOMUTLAR ---
#define BMP180_ADDR                     0x77
#define BMP180_COMMAND_TEMP             0x2E
#define BMP180_COMMAND_PRESS            0x34

// --- REGISTER ADRESLERİ VE UZUNLUKLAR ---
#define BMP180_CONTROL_REG              0xF4
#define BMP180_READ_REG                 0xF6
#define BMP180_CALIB_START_REG          0xAA

#define BMP180_CALIB_DATA_LEN           22
#define BMP180_TEMP_DATA_LEN            2
#define BMP180_PRESS_DATA_LEN           3
#define BMP180_CMD_LEN                  1

// --- GECİKME SÜRELERİ (ms) ---
#define BMP180_TEMP_MEAS_DELAY_MS       5

// --- DURUM VE HATA KODLARI ---
#define BMP180_OK                       0
#define BMP180_ERROR                    -1
#define BMP180_BUSY                     -2

#define BMP180_DEFAULT_SEA_LEVEL_PA     101325.0f

typedef struct {
	int16_t AC1, AC2, AC3;
	uint16_t AC4, AC5, AC6;
	int16_t B1, B2, MB, MC, MD;
} BMP180_Calib_t;

typedef struct{

	float temperature_c;
	float pressure_pa;

	uint16_t utemp;
	uint32_t upressure;

	float altitude_m;

} BMP180_Data_t;

typedef enum {
	BMP180_STATE_WAIT_TEMP, BMP180_STATE_WAIT_PRESS
} BMP180_State_t;

typedef int8_t (*BMP180_I2C_Write_Fn)(uint8_t dev_addr, uint8_t reg_addr,
		const uint8_t *data, uint16_t len, void *user_ctx);

typedef int8_t (*BMP180_I2C_Read_Fn)(uint8_t dev_addr, uint8_t reg_addr,
		uint8_t *data, uint16_t len, void *user_ctx);

typedef uint32_t (*BMP180_GetTick_Fn)(void);

typedef enum{
	OSS0, OSS1, OSS2, OSS3
} OSS;

typedef struct {
	BMP180_I2C_Write_Fn i2c_write;
	BMP180_I2C_Read_Fn i2c_read;
	BMP180_GetTick_Fn get_tick;
	void *user_ctx;

	BMP180_Calib_t calib;

	BMP180_State_t state;
	uint32_t timer;
	long _b5;

	OSS oss;

	float sea_level_pa;

	int8_t data_valid;

	BMP180_Data_t data;
	BMP180_Data_t latest_atomic_data;

} BMP180_Handle_t;

int8_t BMP180_I2C_Init(BMP180_Handle_t *handle, BMP180_I2C_Write_Fn write_fn,
		BMP180_I2C_Read_Fn read_fn, BMP180_GetTick_Fn tick_fn, void *ctx);

int8_t BMP180_Poll(BMP180_Handle_t *handle);

int8_t BMP180_Get_Data(BMP180_Handle_t *h, BMP180_Data_t *out);

void BMP180_Set_Sea_Level_Pressure(BMP180_Handle_t *handle, float pressure_pa);

#ifdef __cplusplus
}
#endif

#endif /* BMP180_H */

