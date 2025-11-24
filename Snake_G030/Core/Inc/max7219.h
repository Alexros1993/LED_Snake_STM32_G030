/*
 * max7219.h
 *
 *  Created on: 22 лист. 2025 р.
 *      Author: oleksandrriznicuk
 */

#ifndef INC_MAX7219_H_
#define INC_MAX7219_H_
#define COLS_VAL 8
#define ROWS_VAL 8

#include <stdint.h>

#include "stm32g0xx_hal.h"

#include "main.h"

void MAX7219_write(uint8_t, uint8_t, SPI_HandleTypeDef* hspi);
void MAX7219_LED_Matrix_fill(int8_t matrix[ROWS_VAL][COLS_VAL], SPI_HandleTypeDef* hspi);
void MAX7219_Init(SPI_HandleTypeDef* hspi);
void MAX7219_Clear_digit_registers(uint8_t min_reg_addr, uint8_t max_reg_addr, SPI_HandleTypeDef* hspi);

#endif /* INC_MAX7219_H_ */
