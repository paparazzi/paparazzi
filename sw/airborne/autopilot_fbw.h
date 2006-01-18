/*
 * $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

/** \file autopilot_fbw.h
 *  \brief Autopilot modes
 *
 */

#ifndef AUTOPILOT_FBW_H
#define AUTOPILOT_FBW_H

#include <inttypes.h>
#include "std.h"
#include "led.h"
#include "paparazzi.h"

#define TRESHOLD_MANUAL_PPRZ (MIN_PPRZ / 2)

#define MODE_MANUAL   0
#define MODE_AUTO     1
#define MODE_FAILSAFE 2
#define MODE_NB       3

#define MODE_OF_PPRZ(mode) ((mode) < TRESHOLD_MANUAL_PPRZ ? MODE_MANUAL : MODE_AUTO)

extern uint8_t autopilot_mode;

#ifdef RADIO_CONTROL
#include "radio_control.h"
static inline void autopilot_process_radio_control ( void ) {
  autopilot_mode = MODE_OF_PPRZ(rc_values[RADIO_MODE]);
  if (autopilot_mode == MODE_MANUAL)
    LED_ON(2);
  else
    LED_OFF(2);
}
#endif /* RADIO_CONTROL */

#endif /* AUTOPILOT_FBW_H */
