/*
 * Parameters.h
 *
 *  Created on: Apr 15, 2025
 *      Author: Lenovo
 */

#ifndef INC_PARAMETERS_H_
#define INC_PARAMETERS_H_

#define NUM_LEDS 			36
#define NUM_DECORATION_LEDS	120
#define NUM_CARS 			4
#define NUM_SPOTS			10
#define NUM_RUNS			3
#define TOLERANCE			15
#define MAX_X				245
#define MAX_Y				205
#define MIN_X				90
#define MIN_Y				55
#define RUN_TIMEOUT			60000
#define DEBOUNCE_TIME_MS	50
#define RANGE_MIN 			0
#define RANGE_MAX 			5



uint16_t x_centers[NUM_LEDS] = {
		230, 206, 179, 154, 124, 101,
		100, 121, 153, 179, 207, 232,
		233, 214, 186, 154, 126, 100,
		99, 123, 154, 185, 209, 231,
		237, 211, 184, 155, 126, 102,
		103, 130, 154, 182, 212, 236
};

uint16_t y_centers[NUM_LEDS] = {
		64, 64, 63, 65, 66, 66,
		91, 91, 90, 90, 90, 89,
		115, 116, 117, 118, 120, 121,
		148, 147, 147, 146, 145, 144,
		174, 174, 176, 177, 175, 177,
		198, 199, 198, 197, 196, 195
};

uint16_t color[NUM_CARS][3] = {{255,0,0},{0,0,255},{0,255,0},{255,165,0}};



#endif /* INC_PARAMETERS_H_ */
