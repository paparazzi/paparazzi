/*
 * Copyright (C) 2012-2013 Freek van Tienen
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/ins/ins_ardrone2.h
 * INS implementation for ardrone2-sdk.
 */

#ifndef INS_ARDRONE2_SDK_H
#define INS_ARDRONE2_SDK_H

#include "subsystems/ins.h"
#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"

struct InsArdrone2 {
  struct LtpDef_i  ltp_def;
  bool_t           ltp_initialized;

  float qfe; ///< not used, only dummy for INS_REF message

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_f ltp_speed;
  struct NedCoor_f ltp_accel;
};

extern struct InsArdrone2 ins_ardrone2;

#define DefaultInsImpl ins_ardrone2
#define InsPeriodic ins_ardrone2_periodic

extern void ins_ardrone2_init(void);
extern void ins_ardrone2_periodic(void);
extern void ins_ardrone2_update_gps(void);

extern void ins_ardrone2_register(void);

#endif /* INS_ARDRONE2_SDK_H */
