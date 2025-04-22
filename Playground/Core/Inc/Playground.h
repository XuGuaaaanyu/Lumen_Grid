/*
 * Playground.h
 *
 *  Created on: Apr 15, 2025
 *      Author: Lenovo
 */

#ifndef INC_PLAYGROUND_H_
#define INC_PLAYGROUND_H_

#include "Parameters.h"
#include <cmath>
#include <ctime>
#include <cstdlib>
typedef struct {
	int x;
	int y;
	bool is_available;
	bool is_occupied;
	int occupied_by;
} ParkingSpot;

typedef struct {
	int x;
	int y;
} Position;

typedef struct {
	uint8_t blue;
	uint8_t green;
	uint8_t red;
	uint8_t brightness;  // 0-31
} APA102_LED;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;


class Car {
	Position p[20]; /*a filter buffer for positions*/
	uint8_t index; /*number of valid p's, 1-based indexing*/
	Position filtered_position;

public:
	Car() {
		for (int i = 0; i < 20; i++) {
			p[i].x = 0;
			p[i].y = 0;
		}
	}

	//EFFECTS: clear the position filter buffer before processing next capture
	//MODIFIES: this->p, this->index
	void clear_position(){
		for (int i = 0; i < 20; i++) {
			p[i].x = 0;
			p[i].y = 0;
		}
		index = 0;
	}

	//EFFECT: add new position (x,y) to the position filter buffer, update index,
	//        and compute the filtered position.
	//MODIFIES: this->p, this->index. this->filtered_position
	void update_position(uint16_t x, uint16_t y){
		if (index == 0) {
			filtered_position.x = x;
			filtered_position.y = y;
			p[0].x = x;
			p[0].y = y;
			index++;
			return;
		}

		// Check if within valid range
		bool valid = (x >= MIN_X && x <= MAX_X && y >= MIN_Y && y <= MAX_Y);

		float alpha = 0.0f;
		if (valid) {
			float dx = x - filtered_position.x;
			float dy = y - filtered_position.y;
			float dist = sqrt(dx * dx + dy * dy);

			// Linearly inverse relation to distance (you can tweak max_dist)
			const float max_dist = 220.0f; // A chosen constant for normalization
			alpha = (dist / max_dist < 1.0f) ? (1.0f - dist / max_dist) : (0.0f);
		}

		// Exponential smoothing
		filtered_position.x = static_cast<int>(filtered_position.x * (1 - alpha) + x * alpha);
		filtered_position.y = static_cast<int>(filtered_position.y * (1 - alpha) + y * alpha);

		// Add to buffer (circular if needed)
		if (index < 20) {
			p[index].x = x;
			p[index].y = y;
			index++;
		} else {
			// Optional: overwrite oldest if full, or just ignore
		}
	}

	Position get_position() const {
		return filtered_position;
	}
};


class Playground {
	ParkingSpot parking_spots[NUM_LEDS];
	APA102_LED leds[NUM_LEDS];
	//APA102_LED decoration[NUM_DECORATION_LEDS];
	int scores[NUM_CARS];
	uint8_t num_occupied;

public:

	//EFFECTS: read in the position of each spot and set all is_available, is_occupied to 0
	void init() {
		for (int i = 0; i < NUM_LEDS; i++) {
			parking_spots[i].x = x_centers[i];
			parking_spots[i].y = y_centers[i];
		}
		/*
		for (int i = 0; i < NUM_DECORATION_LEDS; i++){
			set_led(i, 255, 255, 255, 30);
		}
		APA102_SendFrame();
		*/
	}

	void start_video(){
		uint8_t red[3] = {0, 10, 14};
		uint8_t blue[3] = {5, 7, 15};
		uint8_t green[3] = {30, 28, 20};
		uint8_t yellow[3] = {35, 25, 21};
		for (int i = 0; i < 3; i++){
			set_led(red[i], color[0][0], color[0][1], color[0][2], 15);
			set_led(blue[i], color[1][0], color[1][1], color[1][2], 15);
			set_led(green[i], color[2][0], color[2][1], color[2][2], 15);
			set_led(yellow[i], color[3][0], color[3][1], color[3][2], 15);
			APA102_SendFrame();
			HAL_Delay(200);
		}
		for (int i = 0; i < NUM_CARS; i++){
			scores[i] = 0;
		}
		//clear_leds();
		//APA102_SendFrame();
	}

	void clear(){
		for (int i = 0; i < NUM_LEDS; i++){
			parking_spots[i].is_available = 0;
			parking_spots[i].is_occupied = 0;
			parking_spots[i].occupied_by = -1;
			num_occupied = 0;
		}
		clear_leds();
		APA102_SendFrame();
	}

	//EFFECTS: randomly generates num spots where the spots are marked as is_available
	//MODIFIES: parking_spots.is_available
	void generate_random_spots(uint8_t num) {
		uint8_t count = 0;
		srand(HAL_GetTick());
		while (count < num){
			uint8_t retval = rand() % NUM_LEDS;
			printf("Random Number Generated: %d\r\n", retval);
			if (!parking_spots[retval].is_available){
				parking_spots[retval].is_available = true;

				set_led(retval, 255, 255, 255, 15);
				count ++;
			}
		}
		APA102_SendFrame();
	}

	//EFFECTS: check if spot_id is occupied by a car
	bool check_occupency(uint8_t spot_id, const Position & pos) {
		if (parking_spots[spot_id].is_available
				&& (!parking_spots[spot_id].is_occupied)) {
			if (pos.x - parking_spots[spot_id].x < TOLERANCE
					&& parking_spots[spot_id].x - pos.x < TOLERANCE
					&& pos.y - parking_spots[spot_id].y < TOLERANCE
					&& parking_spots[spot_id].y - pos.y < TOLERANCE) {
				return 1;
			}
		}
		return 0;
	}

	//EEFECTS: car_id occupies spot_id, change the LED color
	//MODIFIES: parking_spots.is_occupied
	void occupy(uint8_t spot_id, uint8_t car_id){
		parking_spots[spot_id].is_occupied = 1;
		parking_spots[spot_id].occupied_by = car_id;

		set_led(spot_id, color[car_id][0], color[car_id][1],  color[car_id][2], 15);
		APA102_SendFrame();

		scores[car_id]++;
		num_occupied++;
	}

	bool is_all_occupied() {
		return num_occupied == NUM_SPOTS;
	}

	void display_result(){
		uint8_t max = 0;
		for(int i = 0; i < NUM_CARS; i++){
			if(scores[max]<scores[i]) max = i;
		}
		for(int i = 0 ; i < NUM_LEDS ; i++){
			set_led(i, color[max][0], color[max][1],  color[max][2], 15);
			APA102_SendFrame();
		}
	}
private:
	void APA102_SendFrame() {
		uint8_t startFrame[4] = { 0x00, 0x00, 0x00, 0x00 };
		HAL_SPI_Transmit(&hspi1, startFrame, 4, HAL_MAX_DELAY);

		for (int i = 0; i < NUM_LEDS; i++) {
			uint8_t frame[4];
			frame[0] = 0b11100000 | (leds[i].brightness & 0x1F); // Brightness frame
			frame[1] = leds[i].blue;
			frame[2] = leds[i].green;
			frame[3] = leds[i].red;
			HAL_SPI_Transmit(&hspi1, frame, 4, HAL_MAX_DELAY);
		}

		uint8_t endFrame[(NUM_LEDS / 16) + 1];
		memset(endFrame, 0xFF, sizeof(endFrame));
		HAL_SPI_Transmit(&hspi1, endFrame, sizeof(endFrame), HAL_MAX_DELAY);
	}

	void set_led(int index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
		if (index >= 0 && index < NUM_LEDS) {
			leds[index].red = r;
			leds[index].green = g;
			leds[index].blue = b;
			leds[index].brightness = brightness;
		}
	}

	void clear_leds() {
		for (int i = 0; i < NUM_LEDS; i++) {
			set_led(i, 0, 0, 0, 0);  // Off
		}
	}
};



#endif /* INC_PLAYGROUND_H_ */
