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
 * @file subsystems/ins/ins_alt_float.c
 * Filters altitude and climb rate for fixedwings.
 */

#include "subsystems/ins/ins_alt_float.h"

#include "subsystems/abi.h"

#include <inttypes.h>
#include <math.h>

#include "state.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/gps.h"
#include "firmwares/fixedwing/nav.h"

#include "generated/airframe.h"
#include "generated/modules.h"

#ifdef DEBUG_ALT_KALMAN
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#endif

#if defined ALT_KALMAN || defined ALT_KALMAN_ENABLED
#warning Please remove the obsolete ALT_KALMAN and ALT_KALMAN_ENABLED defines from your airframe file.
#endif


struct InsAltFloat ins_altf;

#if USE_BAROMETER
#include "subsystems/sensors/baro.h"
#include "math/pprz_isa.h"

PRINT_CONFIG_MSG("USE_BAROMETER is TRUE: Using baro for altitude estimation.")

// Baro event on ABI
#ifndef INS_BARO_ID
#if USE_BARO_BOARD
#define INS_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_BARO_ID)
abi_event baro_ev;
static void baro_cb(uint8_t sender_id, float pressure);
#endif /* USE_BAROMETER */

abi_event gps_ev;

static void alt_kalman_reset(void);
static void alt_kalman_init(void);
static void alt_kalman(float z_meas, float dt);

void ins_alt_float_update_gps(struct GpsState *gps_s);

void ins_alt_float_init(void)
{

  struct UtmCoor_f utm0 = { nav_utm_north0, nav_utm_east0, ground_alt, nav_utm_zone0 };
  stateSetLocalUtmOrigin_f(&utm0);

  stateSetPositionUtm_f(&utm0);

  alt_kalman_init();

#if USE_BAROMETER
  ins_altf.qfe = 0.0f;
  ins_altf.baro_initialized = FALSE;
  ins_altf.baro_alt = 0.0f;
#endif
  ins_altf.reset_alt_ref = FALSE;

  // why do we have this here?
  alt_kalman(0.0f, 0.1);
}

/** Reset the geographic reference to the current GPS fix */
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
  utm.alt = gps.hmsl  / 1000.0f;

  // reset state UTM ref
  stateSetLocalUtmOrigin_f(&utm);

  // reset filter flag
  ins_altf.reset_alt_ref = TRUE;
}

void ins_reset_altitude_ref(void)
{
  struct UtmCoor_f utm = state.utm_origin_f;
  // ground_alt
  utm.alt = gps.hmsl / 1000.0f;
  // reset state UTM ref
  stateSetLocalUtmOrigin_f(&utm);
  // reset filter flag
  ins_altf.reset_alt_ref = TRUE;
}


#if USE_BAROMETER
void ins_alt_float_update_baro(float pressure)
{
  // timestamp in usec when last callback was received
  static uint32_t last_ts = 0;
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  // dt between this and last callback in seconds
  float dt = (float)(now_ts - last_ts) / 1e6;
  last_ts = now_ts;

  // bound dt (assume baro freq 1Hz-500Hz
  Bound(dt, 0.002, 1.0)

  if (!ins_altf.baro_initialized) {
    ins_altf.qfe = pressure;
    ins_altf.baro_initialized = TRUE;
  }
  if (ins_altf.reset_alt_ref) {
    ins_altf.reset_alt_ref = FALSE;
    ins_altf.alt = ground_alt;
    ins_altf.alt_dot = 0.0f;
    ins_altf.qfe = pressure;
    alt_kalman_reset();
  } else { /* not realigning, so normal update with baro measurement */
    ins_altf.baro_alt = ground_alt + pprz_isa_height_of_pressure(pressure, ins_altf.qfe);
    /* run the filter */
    alt_kalman(ins_altf.baro_alt, dt);
    /* set new altitude, just copy old horizontal position */
    struct UtmCoor_f utm;
    UTM_COPY(utm, *stateGetPositionUtm_f());
    utm.alt = ins_altf.alt;
    stateSetPositionUtm_f(&utm);
    struct NedCoor_f ned_vel;
    memcpy(&ned_vel, stateGetSpeedNed_f(), sizeof(struct NedCoor_f));
    ned_vel.z = -ins_altf.alt_dot;
    stateSetSpeedNed_f(&ned_vel);
  }
}
#else
void ins_alt_float_update_baro(float pressure __attribute__((unused)))
{
}
#endif


void ins_alt_float_update_gps(struct GpsState *gps_s)
{
#if USE_GPS
  struct UtmCoor_f utm;
  utm.east = gps_s->utm_pos.east / 100.0f;
  utm.north = gps_s->utm_pos.north / 100.0f;
  utm.zone = nav_utm_zone0;

#if !USE_BAROMETER
#ifdef GPS_DT
  const float dt = GPS_DT;
#else
  // timestamp in usec when last callback was received
  static uint32_t last_ts = 0;
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  // dt between this and last callback in seconds
  float dt = (float)(now_ts - last_ts) / 1e6;
  last_ts = now_ts;

  // bound dt (assume GPS freq between 0.5Hz and 50Hz)
  Bound(dt, 0.02, 2)
#endif

  float falt = gps_s->hmsl / 1000.0f;
  if (ins_altf.reset_alt_ref) {
    ins_altf.reset_alt_ref = FALSE;
    ins_altf.alt = falt;
    ins_altf.alt_dot = 0.0f;
    alt_kalman_reset();
  } else {
    alt_kalman(falt, dt);
    ins_altf.alt_dot = -gps_s->ned_vel.z / 100.0f;
  }
#endif
  utm.alt = ins_altf.alt;
  // set position
  stateSetPositionUtm_f(&utm);

  struct NedCoor_f ned_vel = {
    gps_s->ned_vel.x / 100.0f,
    gps_s->ned_vel.y / 100.0f,
    -ins_altf.alt_dot
  };
  // set velocity
  stateSetSpeedNed_f(&ned_vel);

#endif
}


#define GPS_SIGMA2 1.
#define GPS_R 2.

static float p[2][2];

static void alt_kalman_reset(void)
{
  p[0][0] = 1.0f;
  p[0][1] = 0.0f;
  p[1][0] = 0.0f;
  p[1][1] = 1.0f;
}

static void alt_kalman_init(void)
{
  alt_kalman_reset();
}

static void alt_kalman(float z_meas, float dt)
{
  float R = GPS_R;
  float SIGMA2 = GPS_SIGMA2;

#if USE_BAROMETER
#ifdef SITL
  R = 0.5;
  SIGMA2 = 0.1;
#elif USE_BARO_MS5534A
  if (alt_baro_enabled) {
    R = baro_MS5534A_r;
    SIGMA2 = baro_MS5534A_sigma2;
  }
#elif USE_BARO_ETS
  if (baro_ets_enabled) {
    R = baro_ets_r;
    SIGMA2 = baro_ets_sigma2;
  }
#elif USE_BARO_MS5611
  if (baro_ms5611_enabled) {
    R = baro_ms5611_r;
    SIGMA2 = baro_ms5611_sigma2;
  }
#elif USE_BARO_AMSYS
  if (baro_amsys_enabled) {
    R = baro_amsys_r;
    SIGMA2 = baro_amsys_sigma2;
  }
#elif USE_BARO_BMP
  if (baro_bmp_enabled) {
    R = baro_bmp_r;
    SIGMA2 = baro_bmp_sigma2;
  }
#endif
#endif // USE_BAROMETER

  float q[2][2];
  q[0][0] = dt * dt * dt * dt / 4.;
  q[0][1] = dt * dt * dt / 2.;
  q[1][0] = dt * dt * dt / 2.;
  q[1][1] = dt * dt;


  /* predict */
  ins_altf.alt += ins_altf.alt_dot * dt;
  p[0][0] = p[0][0] + p[1][0] * dt + dt * (p[0][1] + p[1][1] * dt) + SIGMA2 * q[0][0];
  p[0][1] = p[0][1] + p[1][1] * dt + SIGMA2 * q[0][1];
  p[1][0] = p[1][0] + p[1][1] * dt + SIGMA2 * q[1][0];
  p[1][1] = p[1][1] + SIGMA2 * q[1][1];

  /* error estimate */
  float e = p[0][0] + R;

  if (fabs(e) > 1e-5) {
    float k_0 = p[0][0] / e;
    float k_1 =  p[1][0] / e;
    e = z_meas - ins_altf.alt;

    /* correction */
    ins_altf.alt += k_0 * e;
    ins_altf.alt_dot += k_1 * e;

    p[1][0] = -p[0][0] * k_1 + p[1][0];
    p[1][1] = -p[0][1] * k_1 + p[1][1];
    p[0][0] = p[0][0] * (1 - k_0);
    p[0][1] = p[0][1] * (1 - k_0);
  }

#ifdef DEBUG_ALT_KALMAN
  DOWNLINK_SEND_ALT_KALMAN(DefaultChannel, DefaultDevice, &(p[0][0]), &(p[0][1]), &(p[1][0]), &(p[1][1]));
#endif
}

#if USE_BAROMETER
static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure)
{
  ins_alt_float_update_baro(pressure);
}
#endif

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ins_alt_float_update_gps(gps_s);
}

void ins_altf_register(void)
{
  ins_register_impl(ins_alt_float_init);

#if USE_BAROMETER
  // Bind to BARO_ABS message
  AbiBindMsgBARO_ABS(INS_BARO_ID, &baro_ev, baro_cb);
#endif
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);
}
