//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// Arduino UART link class, intended to be used with an Arduino with more than 1 UART, 
// like the Arduino MEGA 2560.  

#ifndef _PIXY2UART_H
#define _PIXY2UART_H

#include "TPixy2.h"
#include "Arduino_Wrapper.h"

#define PIXY_UART_BAUDRATE        19200

class Link2UART {
public:
	int8_t open(uint32_t arg) {
		return 0;
	}

	void close() {
	}

	int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs = NULL) {
		if (cs)
			*cs = 0;

		for (uint8_t i = 0; i < len; i++){
			uint8_t byte = 0;
			HAL_StatusTypeDef ret = HAL_UART_Receive(&huart3, &byte, 1, 2); // timeout = 2ms

			if (ret != HAL_OK){
				return -1;
			}
			buf[i] = byte;

			if (cs)
				*cs += byte;
		}

		return len;
	}

	int16_t send(uint8_t *buf, uint8_t len) {
		if (HAL_UART_Transmit(&huart3, buf, len, HAL_MAX_DELAY) != HAL_OK)
			return -1;
		return len;
	}

private:
	uint8_t m_addr;
};

typedef TPixy2<Link2UART> Pixy2UART;

#endif
