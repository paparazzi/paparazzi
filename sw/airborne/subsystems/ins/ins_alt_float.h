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
 */

/**
 * @file subsystems/ins/ins_alt_float.h
 * Filters altitude and climb rate for fixedwings.
 */

#ifndef INS_ALT_FLOAT_H
#define INS_ALT_FLOAT_H

#include "subsystems/ins.h"

#include <inttypes.h>
#include "std.h"
#include "state.h"
#include "generated/modules.h"

#if USE_BAROMETER
#ifdef BARO_MS5534A
#include "baro_MS5534A.h"
#endif

#if USE_BARO_ETS
#include "modules/sensors/baro_ets.h"
#endif

#if USE_BARO_BMP
#include "modules/sensors/baro_bmp.h"
#endif

#if USE_BARO_MS5611
#include "modules/sensors/baro_ms5611_i2c.h"
#endif

#if USE_BARO_AMSYS
#include "modules/sensors/baro_amsys.h"
#endif

extern float ins_qfe;
extern float ins_baro_alt;
extern bool_t ins_baro_initialized;
#endif //USE_BAROMETER

extern float ins_alt; ///< estimated altitude above MSL in meters

extern float ins_alt_dot; ///< estimated vertical speed in m/s (positive-up)

extern bool_t alt_kalman_enabled;
extern void alt_kalman_reset( void );
extern void alt_kalman_init( void );
extern void alt_kalman( float );

#if USE_BAROMETER
/* Kalman filter cannot be disabled in this mode (no z_dot) */
#define EstimatorSetAlt(z) alt_kalman(z)
#else /* USE_BAROMETER */
#define EstimatorSetAlt(z) { \
  if (!alt_kalman_enabled) { \
    ins_alt = z; \
  } else { \
    alt_kalman(z); \
  } \
}
#endif /* ! USE_BAROMETER */

#endif /* INS_ALT_FLOAT_H */
