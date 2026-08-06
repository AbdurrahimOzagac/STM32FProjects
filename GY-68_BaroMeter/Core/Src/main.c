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
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GY68_ADDRESS_READ	0xEF
#define GY68_ADDRESS_WRITE	0xEE

#define GY68_COMMAND_TEMP  0x2E
#define GY68_COMMAND_PRESS 0x34

#define GY68_CONTROL_REG	0xF4
#define G68_READ_REG		0xF6

#define BMP180_OSS         0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

typedef struct {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4; // Dikkat: Bunlar Unsigned!
	uint16_t AC5; // Dikkat: Bunlar Unsigned!
	uint16_t AC6; // Dikkat: Bunlar Unsigned!
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
} BMP180_Calib_t;

BMP180_Calib_t my_calib;

long b5_global = 0;

uint16_t utemp;
uint32_t upressure;

float temp_c;
long pressure_pa;

long init_pressure_pa;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void BMP180_Read_Calibration(void);
float BMP180_Get_Temperature_Celsius(uint16_t ut, BMP180_Calib_t *calib);
long BMP180_Get_Pressure_Pa(uint32_t up, BMP180_Calib_t *calib);

void BMP180_Read_Calibration(void) {
	uint8_t buffer[22];

	//Read all 22 calibration values at once with i2c
	HAL_I2C_Mem_Read(&hi2c1, GY68_ADDRESS_READ, 0xAA, 1, buffer, 22,
	HAL_MAX_DELAY);

	my_calib.AC1 = (buffer[0] << 8) | buffer[1];
	my_calib.AC2 = (buffer[2] << 8) | buffer[3];
	my_calib.AC3 = (buffer[4] << 8) | buffer[5];
	my_calib.AC4 = (buffer[6] << 8) | buffer[7];
	my_calib.AC5 = (buffer[8] << 8) | buffer[9];
	my_calib.AC6 = (buffer[10] << 8) | buffer[11];
	my_calib.B1 = (buffer[12] << 8) | buffer[13];
	my_calib.B2 = (buffer[14] << 8) | buffer[15];
	my_calib.MB = (buffer[16] << 8) | buffer[17];
	my_calib.MC = (buffer[18] << 8) | buffer[19];
	my_calib.MD = (buffer[20] << 8) | buffer[21];
}

void BMP180_Update() {
	static uint32_t bmp_timer = 0;
	static uint8_t bmp_state = 0;

	switch (bmp_state) {

	case 0:

		//Command to Read Uncompensated Temperature to GY68
		uint8_t command = GY68_COMMAND_TEMP;

		if (HAL_I2C_Mem_Write(&hi2c1, GY68_ADDRESS_WRITE, GY68_CONTROL_REG, 1,
				&command, 1, 100) == HAL_OK) {
			bmp_state = 1;
		}

		bmp_timer = HAL_GetTick();
		break;

	case 1:
		if (HAL_GetTick() - bmp_timer >= 5) {

			//READ Uncompensated Temperature from GY68 Memory
			uint8_t raw_data[2] = { 0 };

			if (HAL_I2C_Mem_Read(&hi2c1, GY68_ADDRESS_READ, G68_READ_REG, 1,
					(uint8_t*) raw_data, 2, 100) == HAL_OK) { //Başarısızlık durumunda yanlış veri verir!
				utemp = (raw_data[0] << 8) | raw_data[1];

				//Command to Read Uncompensated Pressure to GY68
				uint8_t command = GY68_COMMAND_PRESS;

				HAL_I2C_Mem_Write(&hi2c1, GY68_ADDRESS_WRITE, GY68_CONTROL_REG,
						1, &command, 1, 100); //Başarısızlık durumunda yanlış veri verir!

				bmp_state = 2;
				bmp_timer = HAL_GetTick();
			} else
				bmp_state = 0;

		}
		break;

	case 2:
		if (HAL_GetTick() - bmp_timer >= 5) {

			uint8_t raw_data[2] = { 0 };
			if (HAL_I2C_Mem_Read(&hi2c1, GY68_ADDRESS_READ, G68_READ_REG, 1,
					raw_data, 2, 10) == HAL_OK) {

				upressure = (raw_data[0] << 8) | raw_data[1];

				temp_c = BMP180_Get_Temperature_Celsius(utemp, &my_calib);
				pressure_pa = BMP180_Get_Pressure_Pa(upressure, &my_calib);

			}

			bmp_state = 0;

		}
		break;
	}
}

float BMP180_Get_Temperature_Celsius(uint16_t ut, BMP180_Calib_t *calib) {
	long x1, x2;
	long temp_tenth = 0;

	x1 = (((long) ut - (long) calib->AC6) * (long) calib->AC5) >> 15;
	x2 = ((long) calib->MC << 11) / (x1 + (long) calib->MD);

	//Needed for pressure calibration
	b5_global = x1 + x2;

	temp_tenth = (b5_global + 8) >> 4;
	return (float) temp_tenth / 10.0f;
}

long BMP180_Get_Pressure_Pa(uint32_t up, BMP180_Calib_t *calib) {
	long x1, x2, x3, b3, b6;
	unsigned long b4, b7, p;

	b6 = b5_global - 4000;

	x1 = ((long) calib->B2 * ((b6 * b6) >> 12)) >> 11;
	x2 = ((long) calib->AC2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((long) calib->AC1) * 4 + x3) << BMP180_OSS) + 2) >> 2;

	x1 = ((long) calib->AC3 * b6) >> 13;
	x2 = ((long) calib->B1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = ((long) calib->AC4 * (unsigned long) (x3 + 32768)) >> 15;

	b7 = ((unsigned long) up - b3) * (50000 >> BMP180_OSS);

	if (b7 < 0x80000000) {
		p = (b7 << 1) / b4;
	} else {
		p = (b7 / b4) << 1;
	}

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p = p + ((x1 + x2 + 3791) >> 4);

	return p;
}

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
	/* USER CODE BEGIN 2 */

	BMP180_Read_Calibration();

	BMP180_Update();

	init_pressure_pa = pressure_pa;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		BMP180_Update();
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

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
