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

#ifndef DefaultInsImpl
#warning "DefaultInsImpl not set!"
#else
PRINT_CONFIG_VAR(DefaultInsImpl)
#endif

#define __DefaultInsRegister(_x) _x ## _register()
#define _DefaultInsRegister(_x) __DefaultInsRegister(_x)
#define DefaultInsRegister() _DefaultInsRegister(DefaultInsImpl)

/** Inertial Navigation System state */
struct Ins {
  InsInit init;
};

struct Ins ins;

void ins_register_impl(InsInit init)
{
  ins.init = init;

  ins.init();
}

void ins_init(void)
{
  ins.init = NULL;

#ifdef DefaultInsImpl
  DefaultInsRegister();
#endif
}

void ins_init_origin_i_from_flightplan(struct LtpDef_i *ltp_def)
{
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(ltp_def, &ecef_nav0);
  ltp_def->hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(ltp_def);
}


// weak functions, used if not explicitly provided by implementation

void WEAK ins_reset_local_origin(void)
{
#if USE_GPS
  struct UtmCoor_f utm = utm_float_from_gps(&gps, 0);

  // ground_alt
  utm.alt = gps.hmsl  / 1000.0f;

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
    utm->zone = (gps.lla_pos.lon / 1e7 + 180) / 6 + 1;
  }
  utm_of_lla_f(utm, &lla0);

  stateSetLocalUtmOrigin_f(utm);
}
#else
void WEAK ins_reset_utm_zone(struct UtmCoor_f *utm __attribute__((unused))) {}
#endif
