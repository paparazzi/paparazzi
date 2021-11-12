/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
 *
 */

/** @file ins_xsens.c
 * Xsens as a full INS solution
 */

#include "ins_xsens.h"
#include "modules/ins/ins.h"

#include "generated/airframe.h"

#include "mcu_periph/sys_time.h"
#include "modules/core/abi.h"
#include "state.h"

#if USE_GPS_XSENS
#if !USE_GPS
#error "USE_GPS needs to be 1 to use the Xsens GPS!"
#endif
#include "modules/gps/gps.h"
#include "math/pprz_geodetic_float.h"
#include "modules/nav/common_nav.h" /* needed for nav_utm_zone0 */
#endif

/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef INS_XSENS_GPS_ID
#define INS_XSENS_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_XSENS_GPS_ID)
static abi_event gps_ev;

float ins_pitch_neutral;
float ins_roll_neutral;

static void handle_ins_msg(void);
static void update_state_interface(void);
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s);

void ins_xsens_init(void)
{
  xsens_init();

  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;

  struct UtmCoor_f utm0 = { nav_utm_north0, nav_utm_east0, 0., nav_utm_zone0 };
  stateSetLocalUtmOrigin_f(&utm0);
  stateSetPositionUtm_f(&utm0);

  AbiBindMsgGPS(INS_XSENS_GPS_ID, &gps_ev, gps_cb);
}

void ins_xsens_event(void)
{
  xsens_parser_event(&(xsens.parser));
  if (xsens.parser.msg_received) {
    parse_xsens_msg();
    handle_ins_msg();
    xsens.parser.msg_received = FALSE;
  }
}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  struct UtmCoor_f utm = utm_float_from_gps(gps_s, nav_utm_zone0);
  utm.alt = gps_s->hmsl / 1000.;

  // set position
  stateSetPositionUtm_f(&utm);

  struct NedCoor_f ned_vel = {
    gps_s->ned_vel.x / 100.,
    gps_s->ned_vel.y / 100.,
    gps_s->ned_vel.z / 100.
  };
  // set velocity
  stateSetSpeedNed_f(&ned_vel);
}

#if USE_GPS_XSENS
void gps_xsens_init(void)
{
  xsens.gps.nb_channels = 0;
}

static void gps_xsens_publish(void)
{
  // publish gps data
  uint32_t now_ts = get_sys_time_usec();
  xsens.gps.last_msg_ticks = sys_time.nb_sec_rem;
  xsens.gps.last_msg_time = sys_time.nb_sec;
  if (xsens.gps.fix == GPS_FIX_3D) {
    xsens.gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    xsens.gps.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_XSENS_ID, now_ts, &xsens.gps);
}
#endif


static void update_state_interface(void)
{
  // Send to Estimator (Control)
#ifdef XSENS_BACKWARDS
  struct FloatEulers att = {
    -xsens.euler.phi + ins_roll_neutral,
    -xsens.euler.theta + ins_pitch_neutral,
    xsens.euler.psi + RadOfDeg(180)
  };
  struct FloatEulerstRates rates = {
    -xsens.gyro.p,
    -xsens.gyro.q,
    xsens.gyro.r
  };
#else
  struct FloatEulers att = {
    xsens.euler.phi + ins_roll_neutral,
    xsens.euler.theta + ins_pitch_neutral,
    xsens.euler.psi
  };
  struct FloatRates rates = xsens.gyro;
#endif
  stateSetNedToBodyEulers_f(&att);
  stateSetBodyRates_f(&rates);
}


static void handle_ins_msg(void)
{

  update_state_interface();

  if (xsens.new_attitude) {
#ifdef AHRS_TRIGGERED_ATTITUDE_LOOP
    new_ins_attitude = true;
#endif
    xsens.new_attitude = false;
  }

#if USE_GPS_XSENS
  if (xsens.gps_available) {
    // Horizontal speed
    float fspeed = FLOAT_VECT2_NORM(xsens.vel);
    if (xsens.gps.fix != GPS_FIX_3D) {
      fspeed = 0;
    }
    xsens.gps.gspeed = fspeed * 100.;
    xsens.gps.speed_3d = float_vect3_norm(&xsens.vel) * 100;

    float fcourse = atan2f(xsens.vel.y, xsens.vel.x);
    xsens.gps.course = fcourse * 1e7;
    SetBit(xsens.gps.valid_fields, GPS_VALID_COURSE_BIT);

    gps_xsens_publish();
    xsens.gps_available = false;
  }
#endif // USE_GPS_XSENS
}

