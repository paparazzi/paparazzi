/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * @file subsystems/ins.c
 * Integrated Navigation System interface.
 */


#include "subsystems/ins.h"

#if USE_GPS
// for ins_reset_utm_zone
#include "subsystems/gps.h"
#include "state.h"
#endif

struct Ins ins;


// weak functions, used if not explicitly provided by implementation

void WEAK ins_periodic(void) {}

void WEAK ins_reset_local_origin(void) {}

void WEAK ins_reset_altitude_ref(void) {}

#if USE_GPS
void WEAK ins_reset_utm_zone(struct UtmCoor_f * utm) {
  struct LlaCoor_f lla0;
  lla_of_utm_f(&lla0, utm);
#ifdef GPS_USE_LATLONG
  utm->zone = (gps.lla_pos.lon/1e7 + 180) / 6 + 1;
#else
  utm->zone = gps.utm_pos.zone;
#endif
  utm_of_lla_f(utm, &lla0);

  stateSetLocalUtmOrigin_f(utm);
}
#else
void WEAK ins_reset_utm_zone(struct UtmCoor_f * utm __attribute__((unused))) {}
#endif

void WEAK ins_propagate(float dt __attribute__((unused))) {}

void WEAK ins_update_gps(void) {}

