/*
 * Playground.h
 *
 *  Created on: Apr 15, 2025
 *      Author: Lenovo
 */

#ifndef INC_PLAYGROUND_H_
#define INC_PLAYGROUND_H_

class Playground {
	ParkingSpot parking_spots[NUM_LEDS];
	int scores[NUM_CARS];
public:
	Playground();
	virtual ~Playground();
	//EFFECTS: read in the position of each spot and set all is_available, is_occupied to 0
	void init();

	//EFFECTS: randomly generates num spots where the spots are marked as is_available
	//MODIFIES: parking_spots.is_available
	void generate_random_spots(uint8_t num);

	//EFFECTS: check if
	int check_occupency(uint8_t spot_id);

	//EEFECTS: car_id occupies spot_id, change the LED color jidegengxin count++
	//MODIFIES: parking_spots.is_occupied
	void occupy(uint8_t spot_id, uint8_t car_id);

	bool is_all_occupied() {
		return count == NUM_SPOTS;
	}
};

#endif /* INC_PLAYGROUND_H_ */
