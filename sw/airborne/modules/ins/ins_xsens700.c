/*
 * Copyright (C) 2013  Christophe De Wagter
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

/**
 * @file modules/ins/ins_xsens700.c
 * Xsens700 as a full INS solution
 */

#include "ins_xsens700.h"
#include "xsens_common.h"
#include "subsystems/ins.h"

#include "generated/airframe.h"

#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "pprzlink/messages.h"

#if USE_GPS_XSENS
#if !USE_GPS
#error "USE_GPS needs to be 1 to use the Xsens GPS!"
#endif
#include "subsystems/gps.h"
#include "subsystems/abi.h"
#include "math/pprz_geodetic_wgs84.h"
#include "math/pprz_geodetic_float.h"
#include "subsystems/navigation/common_nav.h" /* needed for nav_utm_zone0 */
#endif

/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef INS_XSENS700_GPS_ID
#define INS_XSENS700_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_XSENS700_GPS_ID)
static abi_event gps_ev;

float ins_pitch_neutral;
float ins_roll_neutral;

static void handle_ins_msg(void);
static void update_state_interface(void);
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s);

void ins_xsens700_register(void)
{
  ins_register_impl(ins_xsens700_init);
  AbiBindMsgGPS(INS_XSENS700_GPS_ID, &gps_ev, gps_cb);
}

void ins_xsens700_init(void)
{
  xsens700_init();

  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;

  struct UtmCoor_f utm0 = { nav_utm_north0, nav_utm_east0, 0., nav_utm_zone0 };
  stateSetLocalUtmOrigin_f(&utm0);
  stateSetPositionUtm_f(&utm0);
}

void ins_xsens700_event(void)
{
  xsens_event();
  if (xsens700.msg_received) {
    parse_xsens700_msg();
    handle_ins_msg();
    xsens700.msg_received = false;
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
void gps_xsens700_init(void)
{
  xsens700.gps.nb_channels = 0;
}

static void gps_xsens700_publish(void)
{
  // publish gps data
  uint32_t now_ts = get_sys_time_usec();
  xsens700.gps.last_msg_ticks = sys_time.nb_sec_rem;
  xsens700.gps.last_msg_time = sys_time.nb_sec;
  if (xsens700.gps.fix == GPS_FIX_3D) {
    xsens700.gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    xsens700.gps.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_XSENS_ID, now_ts, &xsens700.gps);
}
#endif


static void update_state_interface(void)
{
  // Send to Estimator (Control)
#ifdef XSENS_BACKWARDS
  struct FloatEulers att = {
    xsens700.euler.phi + ins_roll_neutral,
    -xsens700.euler.theta + ins_pitch_neutral,
    -xsens700.euler.psi + RadOfDeg(180)
  };
  struct FloatEulerstRates rates = {
    xsens700.gyro.p,
    -xsens700.gyro.q,
    -xsens700.gyro.r
  };
#else
  struct FloatEulers att = {
    -xsens700.euler.phi + ins_roll_neutral,
    xsens700.euler.theta + ins_pitch_neutral,
    -xsens700.euler.psi
  };
  struct FloatRates rates  = {
    -xsens700.gyro.p,
    xsens700.gyro.q,
    -xsens700.gyro.r
  };
#endif
  stateSetNedToBodyEulers_f(&att);
  stateSetBodyRates_f(&rates);
}


void handle_ins_msg(void)
{

  update_state_interface();

  if (xsens700.new_attitude) {
    new_ins_attitude = true;
    xsens700.new_attitude = false;
  }

#if USE_GPS_XSENS
  if (xsens700.gps_available) {
    // Horizontal speed
    float fspeed = FLOAT_VECT2_NORM(xsens700.vel);
    if (xsens700.gps.fix != GPS_FIX_3D) {
      fspeed = 0;
    }
    xsens700.gps.gspeed = fspeed * 100.;
    xsens700.gps.speed_3d = float_vect3_norm(&xsens700.vel) * 100;

    float fcourse = atan2f(xsens700.vel.y, xsens700.vel.x);
    xsens700.gps.course = fcourse * 1e7;
    SetBit(xsens700.gps.valid_fields, GPS_VALID_COURSE_BIT);

    gps_xsens700_publish();
    xsens700.gps_available = false;
  }
#endif // USE_GPS_XSENS
}


#ifdef USE_GPS_XSENS
/*
 * register callbacks & structs
 */
void gps_xsens700_register(void)
{
  gps_register_impl(gps_xsens700_init, NULL, GPS_XSENS_ID);
}
#endif
