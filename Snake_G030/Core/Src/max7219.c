/*
 * max7219.c
 *
 *  Created on: 22 лист. 2025 р.
 *      Author: oleksandrriznicuk
 */

#include "max7219.h"

void MAX7219_LED_Matrix_fill(uint8_t matrix[ROWS][COLS], SPI_HandleTypeDef* hspi) {

	//write_to_max7219_from_matrix(ROWS, COLS, MATRIX);
	uint8_t row_data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t cols_addrs[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

	// Fill array from matrix
	for (int8_t j = 0; j < 8; j++) {
		uint8_t data = 0b00000000;

		for (int8_t i = 7; i >= 0; i--) {
			data = data | matrix[i][j] << i;
		}
		row_data[j] = data;
	}

	for (uint8_t i = 0; i < sizeof(cols_addrs); i++) {
		MAX7219_write(cols_addrs[i], row_data[i], hspi);
	}

}

void MAX7219_Init(SPI_HandleTypeDef* hspi) {
	// Set Display to normal mode (Test mode)
	MAX7219_write(0x0F, 0x00, hspi);
	// Set display to normal mode (Shutdown mode)
	MAX7219_write(0x0C, 0x01, hspi);

	// Set display intensity
	MAX7219_write(0x0A, 0x07, hspi);

	MAX7219_write(0x0B, 0x07, hspi);
	MAX7219_write(0x09, 0x00, hspi);
}


void MAX7219_write(uint8_t reg, uint8_t data, SPI_HandleTypeDef* hspi) {
	HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin, GPIO_PIN_RESET);
	uint8_t buf[2] = { reg, data };

	HAL_SPI_Transmit(hspi, buf, sizeof(buf), 1000);
	HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin, GPIO_PIN_SET);
}

void MAX7219_Clear_digit_registers(uint8_t min_reg_addr, uint8_t max_reg_addr, SPI_HandleTypeDef* hspi) {
	for (uint8_t i = min_reg_addr; i <= max_reg_addr; i++) {
		MAX7219_write(i, 0x0, hspi);
	}
}
