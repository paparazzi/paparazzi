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

/** flags for INS reset
 */
#define INS_RESET_REF           0
#define INS_RESET_VERTICAL_REF  1
#define INS_RESET_VERTICAL_POS  2
#define INS_RESET_UTM_ZONE      3


/** initialize the local origin (ltp_def in fixed point) from flight plan position */
extern void ins_init_origin_i_from_flightplan(uint16_t id, struct LtpDef_i *ltp_def);

#endif /* INS_H */
