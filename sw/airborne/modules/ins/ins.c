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
 * @file modules/ins/ins.c
 * Integrated Navigation System interface.
 */


#include "modules/ins/ins.h"

#if USE_GPS
// for ins_reset_utm_zone
#include "modules/gps/gps.h"
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

void WEAK ins_reset_altitude_ref(void) {}

#if USE_GPS
void WEAK ins_reset_utm_zone(struct UtmCoor_f *utm)
{
  struct LlaCoor_f lla0;
  lla_of_utm_f(&lla0, utm);
  if (bit_is_set(gps.valid_fields, GPS_VALID_POS_UTM_BIT)) {
    utm->zone = gps.utm_pos.zone;
  }
  else {
    utm->zone = 0;  // recompute zone from lla
  }
  utm_of_lla_f(utm, &lla0);

  stateSetLocalUtmOrigin_f(utm);
}
#else
void WEAK ins_reset_utm_zone(struct UtmCoor_f *utm __attribute__((unused))) {}
#endif
