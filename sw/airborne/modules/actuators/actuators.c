/*
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
 * Copyright (C) 2012 Gautier Hattenberger
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

/** @file modules/actuators/actuators.c
 *  Hardware independent actuators code.
 *
 */

#include "modules/actuators/actuators.h"
#include "mcu_periph/sys_time.h"

#if ACTUATORS_NB

int16_t actuators[ACTUATORS_NB];

// Can be used to directly control each actuator from the control algorithm
int16_t actuators_pprz[ACTUATORS_NB];

uint32_t actuators_delay_time;
bool   actuators_delay_done;

void actuators_init(void)
{

#if defined ACTUATORS_START_DELAY && ! defined SITL
  actuators_delay_done = false;
  SysTimeTimerStart(actuators_delay_time);
#else
  actuators_delay_done = true;
  actuators_delay_time = 0;
#endif

  // Init macro from generated airframe.h
  AllActuatorsInit();

}

#endif
