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


// weak functions, used if not explicitly provided by implementation

void WEAK ins_reset_local_origin(void)
{
#if USE_GPS
  struct UtmCoor_i utm = utm_int_from_gps(&gps, 0);
  // ground alt
  utm.alt = gps.hmsl;

  // reset state UTM ref
  stateSetLocalUtmOrigin_i(&utm);
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

