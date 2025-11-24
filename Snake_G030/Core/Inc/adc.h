/*
 * adc.h
 *
 *  Created on: 24 лист. 2025 р.
 *      Author: oleksandrriznicuk
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#pragma once
#include <stdint.h>

#include "stm32g0xx_hal.h"


uint16_t read_adc_avg(uint32_t, uint8_t, ADC_HandleTypeDef* );

#endif /* INC_ADC_H_ */
