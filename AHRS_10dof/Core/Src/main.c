/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>

#include "bmp180.h"
#include "bmp180_port_stm32f4.h"
#include "qmc5883p.h"
#include "qmc5883p_port_stm32f4.h"
#include "mpu6050.h"
#include "mpu6050_port_stm32f4.h"
#include "madgwick_ahrs.h"
#include "mahony_ahrs.h"
#include "mag_calibration.h"
#include "telemetry.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

MPU6050_Handle_t mpu6050_handle;
MPU6050_Inertial_t inertial_data;

QMC5883P_Handle_t qmc5883p_handle;
QMC5883P_MAGNETOMETER_t magnetic_data;

BMP180_Handle_t bmp180_handle;
BMP180_Data_t barometer_data;

Madgwick_t madgwick_data;
EulerAngles_t madgwick_euler;

Mahony_t mahony_data;
Mahony_EulerAngles_t mahony_euler;

MagCal_t mag_cal_data;

float pure_compass_yaw1;
float pure_compass_yaw2;

float body_accel_x;
float body_accel_y;
float body_accel_z;

float body_gyro_x;
float body_gyro_y;
float body_gyro_z;

float body_magx;
float body_magy;
float body_magz;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

float Get_Pure_Yaw(float mag_x, float mag_y) {

	/* Calculate heading using arctangent */
	float heading = atan2f(mag_y, mag_x) * (180.0f / 3.14159265f);

	/* Normalize angle to 0 - 360 range */
	if (heading < 0.0f) {
		heading += 360.0f;
	}

	return heading;
}

float Get_Tilt_Compensated_Yaw(float mag_x, float mag_y, float mag_z,
		float roll_deg, float pitch_deg) {

	/* Convert Euler angles from degrees to radians */
	float roll_rad = roll_deg * (3.14159265f / 180.0f);
	float pitch_rad = pitch_deg * (3.14159265f / 180.0f);

	/* Apply tilt compensation formula to project X and Y onto the horizontal plane */
	float comp_mag_x = (mag_x * cosf(pitch_rad)) + (mag_z * sinf(pitch_rad));

	float comp_mag_y = (mag_x * sinf(roll_rad) * sinf(pitch_rad))
			+ (mag_y * cosf(roll_rad))
			- (mag_z * sinf(roll_rad) * cosf(pitch_rad));

	/* Calculate heading using the compensated magnetic values */
	float heading = atan2f(comp_mag_y, comp_mag_x) * (180.0f / 3.14159265f);

	/* Normalize angle to 0 - 360 range */
	if (heading < 0.0f) {
		heading += 360.0f;
	}

	return heading;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim2);
	uint32_t last_micros = __HAL_TIM_GET_COUNTER(&htim2);

	HAL_Delay(500);

	while (1) {

		int8_t err1 = MPU6050_Port_STM32_Init(&mpu6050_handle, &hi2c1);
		int8_t err2 = QMC5883P_Port_STM32_Init(&qmc5883p_handle, &hi2c1);
		int8_t err3 = BMP180_Port_STM32_Init(&bmp180_handle, &hi2c1);

		if (err1 == 0 && err2 == 0 && err3 == 0)
			break;

		HAL_Delay(100);
	}

	MagCal_Start(&mag_cal_data);

	uint32_t cal_start_tick = HAL_GetTick();
	const uint32_t CAL_DURATION_MS = 10;
	const uint16_t CAL_LEDS = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
			| GPIO_PIN_15;

	while ((HAL_GetTick() - cal_start_tick) < CAL_DURATION_MS) {
		QMC5883P_Get_Magnetic_Data(&qmc5883p_handle, &magnetic_data);
		MagCal_Update(&mag_cal_data, magnetic_data.x_mag, magnetic_data.y_mag,
				magnetic_data.z_mag);

		HAL_GPIO_TogglePin(GPIOD, CAL_LEDS);
		HAL_Delay(10);
	}

	HAL_GPIO_WritePin(GPIOD, CAL_LEDS, GPIO_PIN_RESET);

	MagCal_Finish(&mag_cal_data);

	Madgwick_Init(&madgwick_data, 0.4f);
	Mahony_Init(&mahony_data, 2.0f, 0.0f);

	Telemetry_Init(&huart2, 50);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* Convert microseconds to seconds */

		uint32_t current_micros = __HAL_TIM_GET_COUNTER(&htim2);
		uint32_t delta_micros = current_micros - last_micros;
		last_micros = current_micros;

		float dt = (float) delta_micros / 1000000.0f;

		MPU6050_Get_Inertial_Data(&mpu6050_handle, &inertial_data);
		QMC5883P_Get_Magnetic_Data(&qmc5883p_handle, &magnetic_data);

		/* Önce hard-iron/soft-iron kalibrasyonunu uygula */
		float cal_magx, cal_magy, cal_magz;
		MagCal_Apply(&mag_cal_data, magnetic_data.x_mag, magnetic_data.y_mag,
				magnetic_data.z_mag, &cal_magx, &cal_magy, &cal_magz);

		// 1. MPU6050 -> BOARD AXIS CONVERSION
		// Inverting X and Y to fix reversed pitch and roll
		body_accel_x = inertial_data.accel.accel_y;
		body_accel_y = inertial_data.accel.accel_x;
		body_accel_z = inertial_data.accel.accel_z;

		body_gyro_x = inertial_data.gyro.gyro_y;
		body_gyro_y = inertial_data.gyro.gyro_x;
		body_gyro_z = inertial_data.gyro.gyro_z;

		// 2. QMC5883P -> BOARD AXIS CONVERSION
		// Matching magnetometer to the inverted X and Y axes for sensor fusion stability
		// QMC Physical: X=Back, Y=Right, Z=Up
		body_magx = cal_magx;
		body_magy = cal_magy;
		body_magz = cal_magz;

		body_magx = -magnetic_data.y_mag;
		body_magy = -magnetic_data.x_mag;
		body_magz = magnetic_data.z_mag;

		if (dt > 0.0f && dt < 1.0f) {
			Madgwick_Process_Raw_Data(&madgwick_data, body_accel_x,
					body_accel_y, body_accel_z, body_gyro_x, body_gyro_y,
					body_gyro_z, body_magx, body_magy, body_magz, dt);

			Mahony_Process_Raw_Data(&mahony_data, body_accel_x, body_accel_y,
					body_accel_z, body_gyro_x, body_gyro_y, body_gyro_z,
					body_magx, body_magy, body_magz, dt);
		}

		madgwick_euler = QuaternionToEulerAngle(madgwick_data.q);
		mahony_euler = Mahony_QuaternionToEulerAngle(mahony_data.q);

		pure_compass_yaw1 = Get_Tilt_Compensated_Yaw(body_magx, body_magy,
				body_magz, madgwick_euler.roll, madgwick_euler.pitch);

		BMP180_Poll(&bmp180_handle);
		BMP180_Get_Data(&bmp180_handle, &barometer_data);

		Telemetry_Update(madgwick_euler.roll, madgwick_euler.pitch,
				madgwick_euler.yaw, barometer_data.altitude_m);

		HAL_Delay(25);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 83;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pins : PD12 PD13 PD14 PD15 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
