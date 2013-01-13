/*
 * $Id: $
 *
 * Copyright (C) 2012 Sergey Krukowski <softsr@yahoo.de>
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

#ifndef KROOZ_CAM_H
#define KROOZ_CAM_H

#include "generated/airframe.h"
#include "std.h"
//#include "led.h"

#ifdef CAM_SETUP
extern int16_t tilt_center;
extern int16_t tilt_coeff;
extern int16_t pan_coeff;
extern int16_t tilt_rate;
extern int16_t pan_rate;
#endif

extern void krooz_cam_periodic(void);
extern void krooz_cam_event(void);

#endif /* KROOZ2_CAM_H */

