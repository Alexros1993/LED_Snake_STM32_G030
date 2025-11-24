/*
 * game_logic.c
 *
 *  Created on: 24 лист. 2025 р.
 *      Author: oleksandrriznicuk
 */

#include "game_logic.h"

int8_t MATRIX[ROWS][COLS];
uint32_t last_update_time = 0;
const uint32_t display_update_interval = 300;
int8_t snake_dir_x = 0;
int8_t snake_dir_y = 0;

uint8_t snake[64][2];
uint8_t snake_length = 0;

int8_t joy_is_pressed;

void setup_game(SPI_HandleTypeDef *hspi, ADC_HandleTypeDef* hadc) {
	uint32_t last_update_time = 0;
	const uint32_t display_update_interval = 300;
	int8_t snake_dir_x = 0;
	int8_t snake_dir_y = 0;

	uint8_t snake[64][2];
	uint8_t snake_length = 0;

	int8_t joy_is_pressed;


	add_snake_element(snake, &snake_length, 4, 5);
	add_snake_element(snake, &snake_length, 4, 6);
	add_snake_element(snake, &snake_length, 4, 7);

	MAX7219_Clear_digit_registers(0x01, 0x08, hspi);

	fill_matrix(ROWS, COLS, MATRIX, 0);

	for (uint8_t i = 0; i < snake_length; i++) {
		uint8_t x = snake[i][0];
		uint8_t y = snake[i][1];
		MATRIX[y][x] = 1;
	}

	MAX7219_LED_Matrix_fill(MATRIX, hspi);

	snake_dir_y = -1;

	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		joy_controller(&snake_dir_x, &snake_dir_y, &joy_is_pressed, hadc);

		if (HAL_GetTick() - last_update_time >= display_update_interval) {
			last_update_time = HAL_GetTick();

			update_snake_position(snake_dir_x, snake_dir_y, snake,
					snake_length);

			MAX7219_LED_Matrix_fill(MATRIX, hspi);


			fill_matrix(ROWS, COLS, MATRIX, 0);

		}

	}
}

void joy_controller(int8_t *dir_x, int8_t *dir_y, int8_t *is_pressed,
		ADC_HandleTypeDef *hadc) {
	uint16_t joy_x, joy_y;
	uint8_t joy_sw;

	joy_y = read_adc_avg(ADC_CHANNEL_1, 10, hadc);
	joy_x = read_adc_avg(ADC_CHANNEL_0, 10, hadc);
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

void fill_matrix(uint8_t rows, uint8_t cols, uint8_t matrix[rows][cols],
		uint8_t fill_value) {
	for (uint8_t i = 0; i < rows; i++) {
		for (uint8_t j = 0; j < cols; j++) {
			matrix[i][j] = fill_value;
		}
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
