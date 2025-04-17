/*
 * Arduino.h
 *
 *  Created on: Apr 12, 2025
 *      Author: xuguanyu
 */

#ifndef INC_ARDUINO_WRAPPER_H_
#define INC_ARDUINO_WRAPPER_H_

#include <cstring>

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef hlpuart1;

void println(const char * msg){
	HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void delayMicroseconds(uint32_t us){
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
	while ((DWT->CYCCNT - start) < ticks);
}

#endif /* INC_ARDUINO_WRAPPER_H_ */
