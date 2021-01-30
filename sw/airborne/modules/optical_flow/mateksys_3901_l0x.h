/*
 * Copyright (C) 2020 Paparazzi Team
 * 
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/optical_flow/mateksys_3901_l0x.h
 *  @brief Driver for the mateksys_3901_l0x sensor via MSPx protocol output
 *
 */

/*
Since the is no official MSP2 nor MSP1 message definition we choose for time being that
the IDs to be compatible with INAV as see here
https://github.com/iNavFlight/inav/blob/master/src/main/msp/msp_protocol_v2_sensor.h
https://github.com/iNavFlight/inav/wiki/MSP-V2
*/

#ifndef MATEKSYS_3901_L0X_H
#define MATEKSYS_3901_L0X_H

#include "std.h"
#include "stdbool.h"
//#include "filters/median_filter.h" //Who knows we need it ;)

#define MSP2_IS_SENSOR_MESSAGE(x)   ((x) >= 0x1F00 && (x) <= 0x1FFF)

#define MSP2_SENSOR_RANGEFINDER     0x1F01
#define MSP2_SENSOR_OPTIC_FLOW      0x1F02

enum Mateksys3901l0XParseStatus {
  MATEKSYS_3901_L0X_INITIALIZE,               // initialization
  MATEKSYS_3901_L0X_PARSE_HEAD,               
  MATEKSYS_3901_L0X_PARSE_HEAD2,             
  MATEKSYS_3901_L0X_PARSE_DIRECTION,          
  MATEKSYS_3901_L0X_PARSE_LENGTH,           
  MATEKSYS_3901_L0X_PARSE_FUNCTION_ID_B1, 
  MATEKSYS_3901_L0X_PARSE_FUNCTION_ID_B2,     
  MATEKSYS_3901_L0X_PARSE_SIZE,
  MATEKSYS_3901_L0X_PARSE_POINTER,             // ??
  MATEKSYS_3901_L0X_PARSE_DISTANCEQUALITY,     // used if lidar message
  MATEKSYS_3901_L0X_PARSE_DISTANCE_B1,
  MATEKSYS_3901_L0X_PARSE_DISTANCE_B2,
  MATEKSYS_3901_L0X_PARSE_DISTANCE_B3,
  MATEKSYS_3901_L0X_PARSE_DISTANCE_B4,
  MATEKSYS_3901_L0X_PARSE_MOTIONQUALITY,       // used if flow message
  MATEKSYS_3901_L0X_PARSE_MOTIONY_B1,
  MATEKSYS_3901_L0X_PARSE_MOTIONY_B2,
  MATEKSYS_3901_L0X_PARSE_MOTIONY_B3,
  MATEKSYS_3901_L0X_PARSE_MOTIONY_B4,
  MATEKSYS_3901_L0X_PARSE_MOTIONX_B1,
  MATEKSYS_3901_L0X_PARSE_MOTIONX_B2,
  MATEKSYS_3901_L0X_PARSE_MOTIONX_B3,
  MATEKSYS_3901_L0X_PARSE_MOTIONX_B4,
  MATEKSYS_3901_L0X_PARSE_CHECKSUM,        
};

struct Mateksys3901l0X {
  struct link_device *device;
  enum Mateksys3901l0XParseStatus parse_status;
  float  time_sec;
  uint8_t  sensor_id;
	uint8_t  motion_quality;
  int32_t  motionX;
  int32_t  motionY;
  int32_t  motionX_clean;
  int32_t  motionY_clean;
	uint8_t  distancemm_quality;
	int32_t  distancemm;
  float  distance_clean;
  float  velocityX;
  float  velocityY;
	uint8_t  parse_crc; 
};

extern struct Mateksys3901l0X mateksys3901l0x;

extern void mateksys3901l0x_init(void);
extern void mateksys3901l0x_event(void);
extern void mateksys3901l0x_downlink(void);

#endif /* MATEKSYS_3901_L0X_H */

