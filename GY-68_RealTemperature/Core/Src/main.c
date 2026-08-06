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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GY68_ADDRESS_READ	0xEF
#define GY68_ADDRESS_WRITE	0xEE
#define	GY68_COMMAND_TEMP	0x2E

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */


typedef struct {
    int16_t AC5;
    int16_t AC6;
    int16_t MC;
    int16_t MD;
} BMP180_Calib_t;


uint16_t utemp;
float temp_c;

BMP180_Calib_t my_calib;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t BMP180_Read_UT_NonBlocking(uint16_t *out_ut) {
	static uint32_t bmp_start_time = 0;
	static uint8_t bmp_state = 0;

	if (bmp_state == 0) {

		uint8_t command = GY68_COMMAND_TEMP;

		HAL_I2C_Mem_Write(&hi2c1, GY68_ADDRESS_WRITE, 0xF4, 1,
				&command, 1, HAL_MAX_DELAY);

		bmp_start_time = HAL_GetTick();
		bmp_state = 1;
	}

	else if (bmp_state == 1) {
		if (HAL_GetTick() - bmp_start_time >= 5) {

			uint8_t raw_data[2] = { 0 };

			HAL_I2C_Mem_Read(&hi2c1, GY68_ADDRESS_READ, 0xF6, 1, raw_data, 2,
					10);

			*out_ut = (raw_data[0] << 8) | raw_data[1];

			bmp_state = 0;

			return 1;
		}
	}

	return 0;
}
// Ham sıcaklığı (UT) ve katsayıları alıp net Celsius dereceyi döndüren fonksiyon
float BMP180_Get_Temperature_Celsius(uint16_t ut, BMP180_Calib_t *calib) {
    long x1, x2, b5;
    long temp_tenth = 0;

    // Bosch datasheet'inde yer alan resmi formül
    x1 = (((long)ut - (long)calib->AC6) * (long)calib->AC5) >> 15; // Bölü 32768 demek
    x2 = ((long)calib->MC << 11) / (x1 + (long)calib->MD);         // Çarpı 2048 demek
    b5 = x1 + x2;

    // Sonucu 0.1 derece hassasiyetinde hesaplıyoruz (Örn: 253 çıkarsa bu 25.3 derece demektir)
    temp_tenth = (b5 + 8) >> 4;                                   // Bölü 16 demek

    // Float'a çevirip 10'a bölüyoruz ki gerçeği görebilelim
    return (float)temp_tenth / 10.0f;
}

void BMP180_Read_Calibration(void) {
    uint8_t buffer[2] = {0};

    HAL_I2C_Mem_Read(&hi2c1, GY68_ADDRESS_READ, 0xB2, 1, buffer, 2, HAL_MAX_DELAY);
    my_calib.AC5 = (buffer[0] << 8) | buffer[1];

    HAL_I2C_Mem_Read(&hi2c1, GY68_ADDRESS_READ, 0xB4, 1, buffer, 2, HAL_MAX_DELAY);
    my_calib.AC6 = (buffer[0] << 8) | buffer[1];

    HAL_I2C_Mem_Read(&hi2c1, GY68_ADDRESS_READ, 0xBC, 1, buffer, 2, HAL_MAX_DELAY);
    my_calib.MC = (buffer[0] << 8) | buffer[1];

    HAL_I2C_Mem_Read(&hi2c1, GY68_ADDRESS_READ, 0xBE, 1, buffer, 2, HAL_MAX_DELAY);
    my_calib.MD = (buffer[0] << 8) | buffer[1];
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

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (BMP180_Read_UT_NonBlocking(&utemp) == 1) {

			temp_c = BMP180_Get_Temperature_Celsius(utemp, &my_calib);

		}
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
