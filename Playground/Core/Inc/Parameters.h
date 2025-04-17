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
#define NUM_RUNS			1
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
		238, 211, 187, 158, 130, 105,
		101, 126, 154, 184, 214, 236,
		236, 217, 191, 161, 131, 100,
		98, 125, 156, 187, 211, 238,
		237, 210, 181, 156, 130, 101,
		101, 128, 155, 183, 211, 237
};

uint16_t y_centers[NUM_LEDS] = {
		63, 63, 61, 60, 60, 60,
		84, 85, 86, 87, 88, 89,
		116, 115, 116, 114, 115, 114,
		142, 143, 144, 145, 144, 144,
		172, 172, 173, 174, 173, 170,
		195, 196, 197, 197, 195, 195

};

uint16_t color[NUM_CARS][3] = {{255,0,0},{0,0,255},{0,255,0},{255,165,0}};



#endif /* INC_PARAMETERS_H_ */
