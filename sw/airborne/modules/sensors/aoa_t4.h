/*
 * Copyright (C) 2024 The Paparazzi Team
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

/**
 * @file modules/sensors/aoa_t4.h
 * @author Sunyou Hwang, OpenUAS
 * @brief Angle of Attack and optionally Sideslip Angle sensor using T4 Actuators Board and a modified serial bus servo
 * The servo as AOA sensor must be of hall effect type and have motor and gears removed to minimize friction and this solution to work.
 *
 * Read more on how to create your own T4 Board here: https://github.com/tudelft/t4_actuators_board/ 
 * 
 * SENSOR, example : Feetech 3032 Serial Bus Servo
 */

#ifndef AOA_T4_H
#define AOA_T4_H

#include "std.h"

struct Aoa_T4 {
  uint32_t raw; ///< Measurement in degrees times 100 from the sensor before applying direction and offset and unit scale conversion
  float angle;  ///< Angle of attack in radians after applying direction and offset
  float offset; ///< Angle of attack offset in radians
  float filter; ///< Filter level for sensor output, where 0.0 is no filtering and 0.95 for extreme filtering, were 1.0 would output an (useless) constant value.  
};

extern struct Aoa_T4 aoa_t4; // Stores values of angle of attack sensor
/* To allow for separate filter options an own SSA structure is available */
extern struct Aoa_T4 ssa_t4; // Stores values of sideslip angle sensor

/** Selection of sensor type to be send over telemetry for debugging or online logging
 */
enum Aoa_Type {
  SEND_TYPE_AOA,
  SEND_TYPE_SIDESLIP
};
extern enum Aoa_Type aoa_send_type;

/* Compensation Sensor angle to theta angle */
extern float aoa_t4_a1;
extern float aoa_t4_b1;
extern float aoa_t4_a2;
extern float aoa_t4_b2;

void aoa_t4_init(void);
void aoa_t4_update(void);
void aoa_t4_init_filters(void);

#endif /* AOA_T4_H */
