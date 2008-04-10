/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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

/* 
   detect a still vehicle by monitoring variance of imu sensors 
*/ 


#ifndef BOOZ_STILL_DETECTION_H
#define BOOZ_STILL_DETECTION_H

#include "std.h"
#include "6dof.h"

extern float booz_still_detection_accel[AXIS_NB];
extern float booz_still_detection_gyro[AXIS_NB];
extern float booz_still_detection_mag[AXIS_NB];
extern float booz_still_detection_pressure;

extern uint32_t bsd_accel_raw_avg[AXIS_NB];
extern float    bsd_accel_raw_var[AXIS_NB];

extern uint8_t booz_still_detection_status;
#define BSD_STATUS_UNINIT 0
#define BSD_STATUS_LOCKED 1

extern void booz_still_detection_init(void);
extern void booz_still_detection_run(void);

#define BSD_ACCEL_RAW_MAX_VAR 7000.

#endif /* BOOZ_STILL_DETECTION_H */
