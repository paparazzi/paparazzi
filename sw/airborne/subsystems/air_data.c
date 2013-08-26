/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 */

/**
 * @file subsystems/air_data.c
 * Air Data interface
 *  - pressures
 *  - airspeed
 *  - angle of attack and sideslip
 *  - wind
 */

#include "subsystems/air_data.h"
#include "subsystems/abi.h"

/** global AirData state
 */
struct AirData air_data;

/** ABI bindings
 */
#ifndef AIR_DATA_BARO_ABS_ID
#define AIR_DATA_BARO_ABS_ID ABI_BROADCAST
#endif
static abi_event pressure_abs_ev;

static void pressure_abs_cb(uint8_t __attribute__((unused)) sender_id, const float * pressure) {
  air_data.pressure = *pressure;
}

/** AirData initialization. Called at startup.
 *  Bind ABI messages
 */
void air_data_init( void ) {
  AbiBindMsgBARO_ABS(AIR_DATA_BARO_ABS_ID, &pressure_abs_ev, pressure_abs_cb);
}

