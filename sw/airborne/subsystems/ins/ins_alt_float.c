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
#include "subsystems/gps.h"
#include "subsystems/nav.h"

#include "generated/airframe.h"

#ifdef DEBUG_ALT_KALMAN
#include "mcu_periph/uart.h"
#include "ap_downlink.h"
#endif

/* vertical position and speed in meters (z-up)*/
float ins_alt;
float ins_alt_dot;

#if USE_BAROMETER
#include "subsystems/sensors/baro.h"
#include "math/pprz_isa.h"
float ins_qfe;
bool_t  ins_baro_initialized;
float ins_baro_alt;

// Baro event on ABI
#ifndef INS_BARO_ID
#define INS_BARO_ID BARO_BOARD_SENDER_ID
#endif
abi_event baro_ev;
static void baro_cb(uint8_t sender_id, const float *pressure);

#endif

void ins_init() {

  struct UtmCoor_f utm0 = { nav_utm_north0, nav_utm_east0, ground_alt, nav_utm_zone0 };
  stateSetLocalUtmOrigin_f(&utm0);

  stateSetPositionUtm_f(&utm0);

  alt_kalman_init();

#if USE_BAROMETER
  ins_qfe = 0;;
  ins_baro_initialized = FALSE;
  ins_baro_alt = 0.;
  // Bind to BARO_ABS message
  AbiBindMsgBARO_ABS(INS_BARO_ID, &baro_ev, baro_cb);
#endif
  ins.vf_realign = FALSE;

  EstimatorSetAlt(0.);

  ins.status = INS_RUNNING;
}

void ins_periodic( void ) {
}

void ins_realign_h(struct FloatVect2 pos __attribute__ ((unused)), struct FloatVect2 speed __attribute__ ((unused))) {
}

void ins_realign_v(float z __attribute__ ((unused))) {
}

void ins_propagate() {
}

void ins_update_baro() {}

#if USE_BAROMETER
static void baro_cb(uint8_t __attribute__((unused)) sender_id, const float *pressure) {
  if (!ins_baro_initialized) {
    ins_qfe = *pressure;
    ins_baro_initialized = TRUE;
  }
  if (ins.vf_realign) {
    ins.vf_realign = FALSE;
    ins_alt = ground_alt;
    ins_alt_dot = 0.;
    ins_qfe = *pressure;
    alt_kalman_reset();
  }
  else { /* not realigning, so normal update with baro measurement */
    ins_baro_alt = ground_alt + pprz_isa_height_of_pressure(*pressure, ins_qfe);
    /* run the filter */
    EstimatorSetAlt(ins_baro_alt);
    /* set new altitude, just copy old horizontal position */
    struct UtmCoor_f utm;
    UTM_COPY(utm, *stateGetPositionUtm_f());
    utm.alt = ins_alt;
    stateSetPositionUtm_f(&utm);
    struct NedCoor_f ned_vel;
    memcpy(&ned_vel, stateGetSpeedNed_f(), sizeof(struct NedCoor_f));
    ned_vel.z = -ins_alt_dot;
    stateSetSpeedNed_f(&ned_vel);
  }
}
#endif


void ins_update_gps(void) {
#if USE_GPS
  struct UtmCoor_f utm;
  utm.east = gps.utm_pos.east / 100.;
  utm.north = gps.utm_pos.north / 100.;
  utm.zone = nav_utm_zone0;

#if !USE_BAROMETER
  float falt = gps.hmsl / 1000.;
  EstimatorSetAlt(falt);
  if (!alt_kalman_enabled) {
    ins_alt_dot = -gps.ned_vel.z / 100.;
  }
#endif
  utm.alt = ins_alt;
  // set position
  stateSetPositionUtm_f(&utm);

  struct NedCoor_f ned_vel = {
    gps.ned_vel.x / 100.,
    gps.ned_vel.y / 100.,
    -ins_alt_dot
  };
  // set velocity
  stateSetSpeedNed_f(&ned_vel);

#endif
}

void ins_update_sonar() {
}

bool_t alt_kalman_enabled;

#ifndef ALT_KALMAN_ENABLED
#define ALT_KALMAN_ENABLED FALSE
#endif

#ifndef GPS_DT
#define GPS_DT 0.25
#endif
#define GPS_SIGMA2 1.
#define GPS_R 2.

#define BARO_DT 0.1

static float p[2][2];

void alt_kalman_reset( void ) {
  p[0][0] = 1.;
  p[0][1] = 0.;
  p[1][0] = 0.;
  p[1][1] = 1.;
}

void alt_kalman_init( void ) {
  alt_kalman_enabled = ALT_KALMAN_ENABLED;
  alt_kalman_reset();
}

void alt_kalman(float z_meas) {
  float DT;
  float R;
  float SIGMA2;

#if USE_BAROMETER
#ifdef SITL
  DT = BARO_SIM_DT;
  R = 0.5;
  SIGMA2 = 0.1;
#elif USE_BARO_MS5534A
  if (alt_baro_enabled) {
    DT = BARO_DT;
    R = baro_MS5534A_r;
    SIGMA2 = baro_MS5534A_sigma2;
  } else
#elif USE_BARO_ETS
  if (baro_ets_enabled) {
    DT = BARO_ETS_DT;
    R = baro_ets_r;
    SIGMA2 = baro_ets_sigma2;
  } else
#elif USE_BARO_MS5611
  if (baro_ms5611_enabled) {
    DT = BARO_MS5611_DT;
    R = baro_ms5611_r;
    SIGMA2 = baro_ms5611_sigma2;
  } else
#elif USE_BARO_AMSYS
  if (baro_amsys_enabled) {
    DT = BARO_AMSYS_DT;
    R = baro_amsys_r;
    SIGMA2 = baro_amsys_sigma2;
  } else
#elif USE_BARO_BMP
  if (baro_bmp_enabled) {
    DT = BARO_BMP_DT;
    R = baro_bmp_r;
    SIGMA2 = baro_bmp_sigma2;
  } else
#endif
#endif // USE_BAROMETER
  {
    DT = GPS_DT;
    R = GPS_R;
    SIGMA2 = GPS_SIGMA2;
  }

  float q[2][2];
  q[0][0] = DT*DT*DT*DT/4.;
  q[0][1] = DT*DT*DT/2.;
  q[1][0] = DT*DT*DT/2.;
  q[1][1] = DT*DT;


  /* predict */
  ins_alt += ins_alt_dot * DT;
  p[0][0] = p[0][0]+p[1][0]*DT+DT*(p[0][1]+p[1][1]*DT) + SIGMA2*q[0][0];
  p[0][1] = p[0][1]+p[1][1]*DT + SIGMA2*q[0][1];
  p[1][0] = p[1][0]+p[1][1]*DT + SIGMA2*q[1][0];
  p[1][1] = p[1][1] + SIGMA2*q[1][1];

  /* error estimate */
  float e = p[0][0] + R;

  if (fabs(e) > 1e-5) {
    float k_0 = p[0][0] / e;
    float k_1 =  p[1][0] / e;
    e = z_meas - ins_alt;

    /* correction */
    ins_alt += k_0 * e;
    ins_alt_dot += k_1 * e;

    p[1][0] = -p[0][0]*k_1+p[1][0];
    p[1][1] = -p[0][1]*k_1+p[1][1];
    p[0][0] = p[0][0] * (1-k_0);
    p[0][1] = p[0][1] * (1-k_0);
  }

#ifdef DEBUG_ALT_KALMAN
  DOWNLINK_SEND_ALT_KALMAN(DefaultChannel,DefaultDevice,&(p[0][0]),&(p[0][1]),&(p[1][0]), &(p[1][1]));
#endif
}

