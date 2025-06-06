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
 * @file modules/ins/ins_gps_passthrough_utm.c
 *
 * Simply passes GPS UTM position and velocity through to the state interface.
 * For fixedwing firmware since it sets UTM pos only.
 */

#include "modules/ins/ins_gps_passthrough.h"
#include "modules/ins/ins.h"

#include <inttypes.h>
#include <math.h>

#include "state.h"
#include "modules/gps/gps.h"
#include "firmwares/fixedwing/nav.h"


#include "modules/core/abi.h"
/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef INS_PT_GPS_ID
#define INS_PT_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_PT_GPS_ID)
static abi_event gps_ev;

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  struct UtmCoor_f utm = utm_float_from_gps(gps_s, nav_utm_zone0);

  // set position
  stateSetPositionUtm_f(MODULE_INS_GPS_PASSTHROUGH_ID, &utm);

  struct NedCoor_f ned_vel = ned_vel_float_from_gps(gps_s);
  // set velocity
  stateSetSpeedNed_f(MODULE_INS_GPS_PASSTHROUGH_ID, &ned_vel);
}

static abi_event reset_ev;
static void reset_cb(uint8_t sender_id, uint8_t flag);


void ins_gps_passthrough_init(void)
{
  struct UtmCoor_f utm0 = { nav_utm_north0, nav_utm_east0, 0., nav_utm_zone0 };
  stateSetLocalUtmOrigin_f(MODULE_INS_GPS_PASSTHROUGH_ID, &utm0);
  stateSetPositionUtm_f(MODULE_INS_GPS_PASSTHROUGH_ID, &utm0);

  AbiBindMsgGPS(INS_PT_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgINS_RESET(ABI_BROADCAST, &reset_ev, reset_cb);
}

static void reset_ref(void)
{
  struct UtmCoor_f utm = utm_float_from_gps(&gps, 0);
  // reset state UTM ref
  stateSetLocalUtmOrigin_f(MODULE_INS_GPS_PASSTHROUGH_ID, &utm);
}

static void reset_vertical_ref(void)
{
  struct UtmCoor_f utm = *stateGetUtmOrigin_f();
  utm.alt = gps.hmsl / 1000.0f;
  stateSetLocalUtmOrigin_f(MODULE_INS_GPS_PASSTHROUGH_ID, &utm);
}

static void reset_cb(uint8_t sender_id UNUSED, uint8_t flag)
{
  switch (flag) {
    case INS_RESET_REF:
      reset_ref();
      break;
    case INS_RESET_VERTICAL_REF:
      reset_vertical_ref();
      break;
    default:
      // unsupported cases
      break;
  }
}

