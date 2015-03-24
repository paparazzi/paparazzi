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
 * @file subsystems/ins/ins_gps_passthrough_utm.c
 *
 * Simply passes GPS UTM position and velocity through to the state interface.
 * For fixedwing firmware since it sets UTM pos only.
 */

#include "subsystems/ins.h"

#include <inttypes.h>
#include <math.h>

#include "state.h"
#include "subsystems/gps.h"
#include "firmwares/fixedwing/nav.h"

#include "subsystems/ins/ins_gps_passthrough_utm.h"

void ins_gps_utm_init(void)
{
  struct UtmCoor_f utm0 = { nav_utm_north0, nav_utm_east0, 0., nav_utm_zone0 };
  stateSetLocalUtmOrigin_f(&utm0);
  stateSetPositionUtm_f(&utm0);
}

void ins_reset_local_origin(void)
{
  struct UtmCoor_f utm;
#ifdef GPS_USE_LATLONG
  /* Recompute UTM coordinates in this zone */
  struct LlaCoor_f lla;
  LLA_FLOAT_OF_BFP(lla, gps.lla_pos);
  utm.zone = (gps.lla_pos.lon / 1e7 + 180) / 6 + 1;
  utm_of_lla_f(&utm, &lla);
#else
  utm.zone = gps.utm_pos.zone;
  utm.east = gps.utm_pos.east / 100.0f;
  utm.north = gps.utm_pos.north / 100.0f;
#endif
  // ground_alt
  utm.alt = gps.hmsl / 1000.0f;
  // reset state UTM ref
  stateSetLocalUtmOrigin_f(&utm);
}

void ins_reset_altitude_ref(void)
{
  struct UtmCoor_f utm = state.utm_origin_f;
  utm.alt = gps.hmsl / 1000.0f;
  stateSetLocalUtmOrigin_f(&utm);
}


#include "subsystems/abi.h"
static abi_event gps_ev;

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  struct UtmCoor_f utm;
  utm.east = gps_s->utm_pos.east / 100.0f;
  utm.north = gps_s->utm_pos.north / 100.0f;
  utm.zone = nav_utm_zone0;
  utm.alt = gps_s->hmsl / 1000.0f;

  // set position
  stateSetPositionUtm_f(&utm);

  struct NedCoor_f ned_vel = {
    gps_s->ned_vel.x / 100.0f,
    gps_s->ned_vel.y / 100.0f,
    gps_s->ned_vel.z / 100.0f
  };
  // set velocity
  stateSetSpeedNed_f(&ned_vel);
}

void ins_gps_utm_register(void)
{
  ins_register_impl(ins_gps_utm_init);
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);
}
