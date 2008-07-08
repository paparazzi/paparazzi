/*
 * $Id$
 *  
 * Copyright (C) 2004-2006  Pascal Brisset, Antoine Drouin
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

/** \file estimator.h
 * \brief State estimation, fusioning sensors
 */

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <inttypes.h>

#include "std.h"
#include "baro_MS5534A.h"

/* position in meters */
extern float estimator_x;
extern float estimator_y;
extern float estimator_z;

/* attitude in radians */
extern float estimator_phi;
extern float estimator_psi;
extern float estimator_theta;

/* speed in meters per second */
extern float estimator_z_dot;

/* rates in radians per second */
extern float estimator_p;
extern float estimator_q;

/* flight time in seconds */
extern uint16_t estimator_flight_time;
extern float estimator_t;

/* horizontal ground speed in module and dir (m/s, rad (CW/North)) */
extern float estimator_hspeed_mod;
extern float estimator_hspeed_dir;

/* Wind and airspeed estimation sent by the GCS */
extern float wind_east, wind_north; /* m/s */
extern float estimator_airspeed; /* m/s */


void estimator_init( void );
void estimator_propagate_state( void );

void estimator_update_state_gps( void );

extern bool_t alt_kalman_enabled;
#ifdef ALT_KALMAN
extern void alt_kalman_reset( void );
extern void alt_kalman_init( void );
extern void alt_kalman( float );
#endif

#ifdef ALT_KALMAN
#define EstimatorSetPosXY(x, y) { estimator_x = x; estimator_y = y; }

#ifdef USE_BARO_MS5534A
/* Kalman filter cannot be disabled in this mode (no z_dot) */
#define EstimatorSetAlt(z) alt_kalman(z)
#else /* USE_BARO_MS5534A */
#define EstimatorSetAlt(z) { \
  if (!alt_kalman_enabled) { \
    estimator_z = z; \
  } else { \
    alt_kalman(z); \
  } \
}
#endif /* ! USE_BARO_MS5534A */
  
#define EstimatorSetSpeedPol(vhmod, vhdir, vz) { \
  estimator_hspeed_mod = vhmod; \
  estimator_hspeed_dir = vhdir; \
  if (!alt_kalman_enabled) estimator_z_dot = vz; \
}
#else /* ALT_KALMAN */
#define EstimatorSetPosXY(x, y) { estimator_x = x; estimator_y = y; }
#define EstimatorSetAlt(z) { estimator_z = z; }
#define EstimatorSetSpeedPol(vhmod, vhdir, vz) { \
  estimator_hspeed_mod = vhmod; \
  estimator_hspeed_dir = vhdir; \
  estimator_z_dot = vz; \
}

#endif


#define EstimatorSetAtt(phi, psi, theta) { estimator_phi = phi; estimator_psi = psi; estimator_theta = theta; }
#define EstimatorSetPhiPsi(phi, psi) { estimator_phi = phi; estimator_psi = psi; }

#define EstimatorSetRate(p, q) { estimator_p = p; estimator_q = q; }

#endif /* ESTIMATOR_H */
