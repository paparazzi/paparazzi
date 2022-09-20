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
 * @file modules/ins/ins.h
 * Integrated Navigation System interface.
 */

#ifndef INS_H
#define INS_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"
#include "state.h"

/* underlying includes (needed for parameters) */
#ifdef INS_TYPE_H
#include INS_TYPE_H
#endif


/** INS local origin reset.
 *  Reset horizontal and vertical reference to the current position.
 *  Does nothing if not implemented by specific INS algorithm.
 */
extern void ins_reset_local_origin(void);

/** INS altitude reference reset.
 *  Reset only vertical reference to the current altitude.
 *  Does nothing if not implemented by specific INS algorithm.
 */
extern void ins_reset_altitude_ref(void);

/** INS vertical position reset.
 *  Reset only vertical position to zero.
 *  Does nothing if not implemented by specific INS algorithm.
 */
extern void ins_reset_vertical_pos(void);

/** INS utm zone reset.
 *  Reset UTM zone according the the actual position.
 *  Only used with fixedwing firmware.
 *  Can be overwritte by specifc INS implementation.
 *  @param utm initial utm zone, returns the corrected utm position
 */
extern void ins_reset_utm_zone(struct UtmCoor_f *utm);

/** initialize the local origin (ltp_def in fixed point) from flight plan position */
extern void ins_init_origin_i_from_flightplan(struct LtpDef_i *ltp_def);


#endif /* INS_H */
