/*
 * obstacle_message.h
 *
 *  Created on: Mar 3, 2022
 *      Author: frederic
 */

#include "std.h"

#ifndef SW_AIRBORNE_MODULES_COMPUTER_VISION_OBSTACLE_MESSAGE_H_
#define SW_AIRBORNE_MODULES_COMPUTER_VISION_OBSTACLE_MESSAGE_H_

struct ObstacleMessage {
	int16_t pos_x;
	int16_t pos_y;
	int16_t obs_width;
	int16_t obs_height;
	int16_t quality;
	int16_t time2impact;
	int16_t options;
};


#endif /* SW_AIRBORNE_MODULES_COMPUTER_VISION_OBSTACLE_MESSAGE_H_ */
