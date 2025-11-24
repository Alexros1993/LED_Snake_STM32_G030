/*
 * game_logic.h
 *
 *  Created on: 24 лист. 2025 р.
 *      Author: oleksandrriznicuk
 */

#ifndef INC_GAME_LOGIC_H_
#define INC_GAME_LOGIC_H_


#define SNAKE_START_POS_X 4
#define SNAKE_START_POS_Y 7

#pragma once
#include <string.h>
#include <stdint.h>

#include "stm32g0xx_hal.h"
#include "max7219.h"

void setup_game(SPI_HandleTypeDef *hspi, ADC_HandleTypeDef* hadc);

void run_game(ADC_HandleTypeDef* hadc, SPI_HandleTypeDef* hspi);

void joy_controller(int8_t *dir_x, int8_t *dir_y, int8_t *is_pressed, ADC_HandleTypeDef* hadc);

void fill_matrix(uint8_t rows, uint8_t cols, uint8_t matrix[rows][cols],
		uint8_t fill_value);

void add_snake_element(uint8_t array[64][2], uint8_t *array_length, uint8_t x,
		uint8_t y);

void update_snake_position(int8_t snake_dir_x, int8_t snake_dir_y,
		uint8_t snake[][2], uint8_t snake_length);

#endif /* INC_GAME_LOGIC_H_ */
