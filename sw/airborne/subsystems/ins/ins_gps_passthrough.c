/*
 * Copyright (C) 2004-2012 The Paparazzi Team
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
 * @file subsystems/ins/ins_gps_passthrough.c
 *
 * Simply passes GPS position and velocity through to the state interface.
 */

#include "subsystems/ins.h"

#include <inttypes.h>
#include <math.h>

#include "state.h"
#include "subsystems/gps.h"
#include "subsystems/nav.h"

void ins_init() {
  struct UtmCoor_f utm0 = { nav_utm_north0, nav_utm_east0, 0., nav_utm_zone0 };
  stateSetLocalUtmOrigin_f(&utm0);
  stateSetPositionUtm_f(&utm0);

  ins.status = INS_RUNNING;
}

void ins_periodic( void ) {
}

void ins_update_gps(void) {
  struct UtmCoor_f utm;
  utm.east = gps.utm_pos.east / 100.;
  utm.north = gps.utm_pos.north / 100.;
  utm.zone = nav_utm_zone0;
  utm.alt = gps.hmsl / 1000.;

  // set position
  stateSetPositionUtm_f(&utm);

  struct NedCoor_f ned_vel = {
    gps.ned_vel.x / 100.,
    gps.ned_vel.y / 100.,
    gps.ned_vel.z / 100.
  };
  // set velocity
  stateSetSpeedNed_f(&ned_vel);
}


void ins_propagate() {
}

void ins_update_baro() {
}

void ins_update_sonar() {
}

void ins_realign_h(struct FloatVect2 pos __attribute__ ((unused)), struct FloatVect2 speed __attribute__ ((unused))) {
}

void ins_realign_v(float z __attribute__ ((unused))) {
}

