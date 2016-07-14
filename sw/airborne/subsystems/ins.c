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

#include "generated/flight_plan.h"


void ins_init_origin_i_from_flightplan(struct LtpDef_i *ltp_def)
{
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  ltp_def_from_lla_i(ltp_def, &llh_nav0);
  ltp_def->hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(ltp_def);

  struct UtmCoor_f utm_def = { NAV_UTM_NORTH0, NAV_UTM_EAST0, GROUND_ALT, NAV_UTM_ZONE0 };
  stateSetLocalUtmOrigin_f(&utm_def);
}


// weak functions, used if not explicitly provided by implementation

void WEAK ins_reset_local_origin(void)
{
#if USE_GPS
  struct UtmCoor_f utm = utm_float_from_gps(&gps, 0);

  // reset state UTM ref
  stateSetLocalUtmOrigin_f(&utm);
#endif
}

void WEAK ins_reset_altitude_ref(void)
{
#if USE_GPS
  struct UtmCoor_f utm;
  UTM_COPY(utm, state.utm_origin_f);
  utm.alt = gps.hmsl / 1000.0f;
  stateSetLocalUtmOrigin_f(&utm);
#endif
}

#if USE_GPS
void WEAK ins_reset_utm_zone(uint8_t zone)
{
  struct LlaCoor_f lla0;
  struct UtmCoor_f utm0;
  UTM_COPY(utm0, state.utm_origin_f);
  lla_of_utm_f(&lla0, &utm0);
  if (zone > 0) {
    utm0.zone = zone;  // recompute utm zone from gps
  } else {
    utm0.zone = utm_float_from_gps(&gps, 0).zone;  // recompute utm zone from gps
  }
  utm_of_lla_f(&utm0, &lla0);

  stateSetLocalUtmOrigin_f(&utm0);
}
#else
void WEAK ins_reset_utm_zone(uint8_t zone __attribute__((unused))) {}
#endif
