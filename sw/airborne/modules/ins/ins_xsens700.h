/*
 * Copyright (C) 2010 ENAC
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

/**
 * @file modules/ins/ins_xsens700.h
 * Xsens700 as a full INS solution
 */

#ifndef INS_XSENS700_H
#define INS_XSENS700_H

#include "std.h"

#include "xsens700.h"

#ifdef AHRS_TRIGGERED_ATTITUDE_LOOP
extern volatile uint8_t new_ins_attitude;
#endif

extern float ins_pitch_neutral;
extern float ins_roll_neutral;

extern void ins_xsens700_init(void);
extern void ins_xsens700_event(void);

#if USE_GPS_XSENS
#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_XSENS
#endif
extern void gps_xsens700_init(void);
#endif

#endif
