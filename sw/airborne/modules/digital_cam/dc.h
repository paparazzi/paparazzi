/*
 * Copyright (C) 2010 The Paparazzi Team
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


/** \file dc.h
 *  \brief Standard Digital Camera Control Interface
 *
 * 	-Standard IO
 * 	-I2C Control
 *
 *  Usage: (from the flight plan, the settings or any airborne code):
 *  - dc_send_command(  )
 *  - set the appropriate autoshoot mode (off/time/distance/trigger)
 *  - use the module periodic function to set the autorepeat interval
 *  - define SENSOR_SYNC_SEND to get the DC_SHOT_MESSAGE on every SHOOT command
 */

#ifndef DC_H
#define DC_H

#include "std.h"
#include "led.h"
#include "generated/airframe.h"
#include "subsystems/gps.h"


/* Generic Set of Digital Camera Commands */
typedef enum {
	DC_GET_STATUS = 0,

	DC_HOLD = 13,
	DC_SHOOT = 32,

	DC_WIDER = 'w',
	DC_TALLER = 't',

	DC_UP = 'u',
	DC_DOWN = 'd',
	DC_CENTER = 'c',
	DC_LEFT = 'l',
	DC_RIGHT = 'r',

	DC_MENU = 'm',
	DC_HOME = 'h',
	DC_PLAY = 'p',

	DC_ON = 'O',
	DC_OFF = 'o',

} dc_command_type;

/* Send Command To Camera */
static inline void dc_send_command(uint8_t cmd);

/* Auotmatic Digital Camera Photo Triggering */
typedef enum {
	DC_AUTOSHOOT_STOP = 0,
	DC_AUTOSHOOT_PERIODIC = 1,
	DC_AUTOSHOOT_DISTANCE = 2,
	DC_AUTOSHOOT_EXT_TRIG = 3
} dc_autoshoot_type;
extern dc_autoshoot_type dc_autoshoot;

/* AutoShoot photos every X quarter_second */
extern uint8_t dc_autoshoot_quartersec_period;

/* AutoShoot photos on a X meter Local Tangent Plane Grid */
extern uint8_t dc_autoshoot_meter_grid;

/* Send Down the coordinates of where the photo was taken */
#ifdef SENSOR_SYNC_SEND
void dc_send_shot_position(void);
#else
#define dc_send_shot_position() {}
#endif

/******************************************************************
 * FUNCTIONS
 *****************************************************************/

/* get settings */
static inline void dc_init(void)
{
#ifdef DC_AUTOSHOOT_QUARTERSEC_PERIOD
  dc_autoshoot_quartersec_period = DC_AUTOSHOOT_QUARTERSEC_PERIOD;
#endif
#ifdef DC_AUTOSHOOT_METER_GRID
  dc_autoshoot_meter_grid = DC_AUTOSHOOT_METER_GRID;
#endif
}

/* shoot on grid */
static inline void dc_shot_on_utm_north_close_to_100m_grid( void )
{
  uint32_t dist_to_100m_grid = (gps.utm_pos.north / 100) % 100;
  if (dist_to_100m_grid < dc_autoshoot_meter_grid || 100 - dist_to_100m_grid < dc_autoshoot_meter_grid)
  {
      dc_send_command(DC_SHOOT);
  }
}

/* periodic 4Hz function */
static inline void dc_periodic_4Hz( void )
{
  static uint8_t dc_shutter_timer = 0;

#ifdef DC_AUTOSHOOT_QUARTERSEC_PERIOD
  if (dc_autoshoot == DC_AUTOSHOOT_PERIODIC)
  {
    if (dc_shutter_timer)
    {
      dc_shutter_timer--;
    } else {
      dc_send_command(DC_SHOOT);
      dc_shutter_timer = dc_autoshoot_quartersec_period;
    }
  }
#endif
#ifdef DC_AUTOSHOOT_METER_GRID
  if (dc_autoshoot == DC_AUTOSHOOT_DISTANCE)
  {
    // Shoot
    dc_shot_on_utm_north_close_to_100m_grid();
  }
#endif
}




#endif // DC_H
