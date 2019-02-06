/*
 * Copyright (C) Fabien Bonneval
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/actuators/actuators_ostrich.h"
 * @author Fabien Bonneval
 * Driver for the Ostrich rover controller board
 */

#ifndef ACTUATORS_OSTRICH_H
#define ACTUATORS_OSTRICH_H

#include "std.h"

#define SPEED_NEUTRAL 500
#define SPEED_FACTOR 1.0
#define TURN_NEUTRAL 500
#define TURN_FACTOR 0.5

#define START_BYTE 0x7F

/* Main actuator structure */
struct ActuatorsOstrich {
  uint16_t cmds[3];      ///< commands
  //int32_t encoders[2];  ///< encoder values
  //bool initialized;     ///< init flag
};

struct SpeedMessagePayload {
  uint16_t vx;
  uint16_t vy;
  uint16_t vtheta;
} __attribute__((packed)) ;

union rawData {
  struct SpeedMessagePayload data;
  uint8_t bytes[6];
} __attribute__((packed)) ;

struct SpeedMessage  {
  uint8_t start_byte;
  uint8_t msg_type;
  uint8_t checksum;
  union rawData raw_data;
} __attribute__((packed)) ;

union RawMessage {
  struct SpeedMessage speed_message;
  uint8_t bytes[9];
} __attribute__((packed)) ;

extern struct ActuatorsOstrich actuators_ostrich;

extern void actuators_ostrich_init(void);
extern void actuators_ostrich_periodic(void);
extern void actuators_ostrich_event(void);
//extern void actuators_ostrich_set(void);

/* Actuator macros */
#define ActuatorOSTRICHSet(_i, _v) { actuators_ostrich.cmds[_i] = _v; }
#define ActuatorsOSTRICHInit() actuators_ostrich_init()
#define ActuatorsOSTRICHCommit() {}

#endif

