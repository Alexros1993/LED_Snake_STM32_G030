/*
 * adc.c
 *
 *  Created on: 24 лист. 2025 р.
 *      Author: oleksandrriznicuk
 */

#include "adc.h"

uint16_t read_adc(uint32_t channel, ADC_HandleTypeDef* hadc) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;

	HAL_ADC_ConfigChannel(hadc, &sConfig);

	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	uint16_t value = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop(hadc);

	return value;

}

uint16_t read_adc_avg(uint32_t channel, uint8_t samples, ADC_HandleTypeDef* hadc) {
	uint32_t sum = 0;
	for (uint8_t i = 0; i < samples; i++) {
		sum += read_adc(channel, hadc);
	}
	return sum / samples;
}
