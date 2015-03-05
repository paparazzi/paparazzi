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

/** Ins implementation state (altitude, float) */
struct InsAltFloat {
  float alt;     ///< estimated altitude above MSL in meters
  float alt_dot; ///< estimated vertical speed in m/s (positive-up)

  bool_t reset_alt_ref;  ///< flag to request reset of altitude reference to current alt

#if USE_BAROMETER
  float qfe;
  float baro_alt;
  bool_t baro_initialized;
#endif
};

extern struct InsAltFloat ins_altf;

extern void ins_alt_float_init(void);
extern void ins_alt_float_update_baro(float pressure);

#ifndef DefaultInsImpl
#define DefaultInsImpl ins_altf
#endif
extern void ins_altf_register(void);

#endif /* INS_ALT_FLOAT_H */
