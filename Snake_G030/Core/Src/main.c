/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COLS 8
#define ROWS 8
#define SNAKE_START_POS_X 4
#define SNAKE_START_POS_Y 7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t MATRIX[ROWS][COLS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t read_adc(uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;

	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t value = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	return value;

}

uint16_t read_adc_avg(uint32_t channel, uint8_t samples) {
	uint32_t sum = 0;
	for (uint8_t i = 0; i < samples; i++) {
		sum += read_adc(channel);
	}
	return sum / samples;
}

void max7219_write(uint8_t reg, uint8_t data) {
	HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin, GPIO_PIN_RESET);
	uint8_t buf[2] = { reg, data };

	HAL_SPI_Transmit(&hspi1, buf, sizeof(buf), 1000);
	HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin, GPIO_PIN_SET);
}

void joy_controller(int8_t *dir_x, int8_t *dir_y, int8_t *is_pressed) {
	uint16_t joy_x, joy_y;
	uint8_t joy_sw;

	joy_y = read_adc_avg(ADC_CHANNEL_1, 10);
	joy_x = read_adc_avg(ADC_CHANNEL_0, 10);
	joy_sw = HAL_GPIO_ReadPin(GPIOA, SW_Pin);

	if (joy_x < 100 && *dir_x == 0) {
		*dir_x = -1;
		*dir_y = 0;
	} else if (joy_x > 3700 && *dir_x == 0) {
		*dir_x = 1;
		*dir_y = 0;
	} else if (joy_y < 100 && *dir_y == 0) {
		*dir_y = -1;
		*dir_x = 0;
	} else if (joy_y > 3700 && *dir_y == 0) {
		*dir_y = 1;
		*dir_x = 0;
	}

	if (joy_sw == 0) {
		*is_pressed = 1;
	} else {
		*is_pressed = 0;
	}
}

void MAX7219_Init() {
	// Set Display to normal mode (Test mode)
	max7219_write(0x0F, 0x00);
	// Set display to normal mode (Shutdown mode)
	max7219_write(0x0C, 0x01);

	// Set display intensity
	max7219_write(0x0A, 0x07);

	max7219_write(0x0B, 0x07);
	max7219_write(0x09, 0x00);
}

void fill_matrix(uint8_t rows, uint8_t cols, uint8_t matrix[rows][cols],
		uint8_t fill_value) {
	for (uint8_t i = 0; i < rows; i++) {
		for (uint8_t j = 0; j < cols; j++) {
			matrix[i][j] = fill_value;
		}
	}
}

void write_to_max7219_from_matrix(uint8_t rows, uint8_t cols,
		uint8_t matrix[rows][cols]) {
	for (uint8_t i = 0; i < rows; i++) {
		for (uint8_t j = 0; j < cols; j++) {
			if (matrix[i][j] == 1) {
				uint8_t row_data = 0b01 << i;
				uint8_t col_addr = 0x0 + j + 1;
				max7219_write(col_addr, row_data);
			}
		}
	}
}

void MAX7219_Clear_digit_registers(uint8_t min_reg_addr, uint8_t max_reg_addr) {
	for (uint8_t i = min_reg_addr; i <= max_reg_addr; i++) {
		max7219_write(i, 0x00);
	}
}

void add_snake_element(uint8_t array[64][2], uint8_t *array_length, uint8_t x,
		uint8_t y) {
	array[*array_length][0] = x;
	array[*array_length][1] = y;
	*array_length += 1;
}

void update_snake_position(int8_t snake_dir_x, int8_t snake_dir_y,
		uint8_t snake[][2], uint8_t snake_length) {
	uint8_t prev_x;
	uint8_t prev_y;

	for (uint8_t i = 0; i < snake_length; i++) {
		if (i == 0) {
			prev_x = snake[i][0];
			prev_y = snake[i][1];

			int8_t x = snake[i][0] + snake_dir_x;
			int8_t y = snake[i][1] + snake_dir_y;

			if (x > 7) {
				x = 0;
			} else if (x < 0) {
				x = 7;
			}

			if (y > 7) {
				y = 0;
			} else if (y < 0) {
				y = 7;
			}

			snake[i][0] = x;
			snake[i][1] = y;

			MATRIX[snake[i][1]][snake[i][0]] = 1;

		} else {
			int8_t x = snake[i][0];
			int8_t y = snake[i][1];

			snake[i][0] = prev_x;
			snake[i][1] = prev_y;

			MATRIX[snake[i][1]][snake[i][0]] = 1;

			prev_x = x;
			prev_y = y;
			MATRIX[prev_y][prev_x] = 0;
		}

	}

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
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */
	MAX7219_Init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	uint32_t last_update_time = 0;
	const uint32_t display_update_interval = 300;

	int8_t snake_dir_x = 0;
	int8_t snake_dir_y = 0;

	uint8_t snake[64][2];
	uint8_t snake_length = 0;

	add_snake_element(snake, &snake_length, 4, 3);
	add_snake_element(snake, &snake_length, 4, 4);
	add_snake_element(snake, &snake_length, 4, 5);
	add_snake_element(snake, &snake_length, 4, 6);
	add_snake_element(snake, &snake_length, 4, 7);

	int8_t joy_is_pressed;

	uint8_t buffer[40];

	MAX7219_Clear_digit_registers(0x01, 0x08);

	fill_matrix(ROWS, COLS, MATRIX, 0);

	for (uint8_t i = 0; i < snake_length; i++) {
		uint8_t x = snake[i][0];
		uint8_t y = snake[i][1];
		MATRIX[y][x] = 1;
	}

	uint8_t row_data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	// Fill array from matrix
	for (int8_t j = 0; j < 8; j++) {
		uint8_t data = 0b00000000;

		for (int8_t i = 7; i >= 0; i--) {
			data = data | MATRIX[i][j] << i;
		}
		row_data[j] = data;
	}

	max7219_write(0x01, row_data[0]);
	max7219_write(0x02, row_data[1]);
	max7219_write(0x03, row_data[2]);
	max7219_write(0x04, row_data[3]);
	max7219_write(0x05, row_data[4]);
	max7219_write(0x06, row_data[5]);
	max7219_write(0x07, row_data[6]);
	max7219_write(0x08, row_data[7]);

	snake_dir_y = -1;

	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		joy_controller(&snake_dir_x, &snake_dir_y, &joy_is_pressed);

		if (HAL_GetTick() - last_update_time >= display_update_interval) {
			last_update_time = HAL_GetTick();

			update_snake_position(snake_dir_x, snake_dir_y, snake,
					snake_length);

			//write_to_max7219_from_matrix(ROWS, COLS, MATRIX);
			uint8_t row_data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

			// Fill array from matrix
			for (int8_t j = 0; j < 8; j++) {
				uint8_t data = 0b00000000;

				for (int8_t i = 7; i >= 0; i--) {
					data = data | MATRIX[i][j] << i;
				}
				row_data[j] = data;
			}

			max7219_write(0x01, row_data[0]);
			max7219_write(0x02, row_data[1]);
			max7219_write(0x03, row_data[2]);
			max7219_write(0x04, row_data[3]);
			max7219_write(0x05, row_data[4]);
			max7219_write(0x06, row_data[5]);
			max7219_write(0x07, row_data[6]);
			max7219_write(0x08, row_data[7]);

			snprintf(buffer, sizeof(buffer), "X: %d, Y: %d, BTN: %d \r\n",
					snake_dir_x, snake_dir_y, joy_is_pressed);

			HAL_UART_Transmit(&huart1, buffer, sizeof(buffer), 1000);

			fill_matrix(ROWS, COLS, MATRIX, 0);

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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SW_Pin */
	GPIO_InitStruct.Pin = SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_CS_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
