/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file stabilization_rate_indi.c
 *  Rate stabilization for rotorcrafts based on INDI by Ewoud Smeur.
 */
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

void stabilization_rate_init(void)
{
  stabilization_indi_init();
}


void stabilization_rate_read_rc(void)
{
  //FIXME: make a new indi function
}

//Read rc with roll and yaw sitcks switched if the default orientation is vertical but airplane sticks are desired
void stabilization_rate_read_rc_switched_sticks(void)
{
  //FIXME: make a new indi function
}

void stabilization_rate_enter(void)
{
  stabilization_indi_enter();
}

void stabilization_rate_run(bool in_flight)
{
  stabilization_indi_run(in_flight, TRUE);
}
