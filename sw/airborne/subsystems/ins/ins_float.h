/*
 * Copyright (C) 2004-2006  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2012  Gautier Hattenberger
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

#ifndef INS_FLOAT_H
#define INS_FLOAT_H

#include "subsystems/ins.h"

#include <inttypes.h>
#include "std.h"
#include "state.h"

#ifdef BARO_MS5534A
#include "baro_MS5534A.h"
#endif

#if USE_BARO_ETS
#include "modules/sensors/baro_ets.h"
#endif

#if USE_BARO_BMP
#include "modules/sensors/baro_bmp.h"
#endif


/* position in meters, ENU frame, relative to reference */
extern float estimator_z; ///< altitude above MSL in meters

/* speed in meters per second */
extern float estimator_z_dot;

extern bool_t alt_kalman_enabled;
#ifdef ALT_KALMAN
extern void alt_kalman_reset( void );
extern void alt_kalman_init( void );
extern void alt_kalman( float );
#endif

#ifdef ALT_KALMAN

#if USE_BARO_MS5534A || USE_BARO_ETS || USE_BARO_BMP
/* Kalman filter cannot be disabled in this mode (no z_dot) */
#define EstimatorSetAlt(z) alt_kalman(z)
#else /* USE_BARO_x */
#define EstimatorSetAlt(z) { \
  if (!alt_kalman_enabled) { \
    estimator_z = z; \
  } else { \
    alt_kalman(z); \
  } \
}
#endif /* ! USE_BARO_x */

#else /* ALT_KALMAN */
#define EstimatorSetAlt(z) { estimator_z = z; }
#endif

#endif /* INS_FLOAT_H */
