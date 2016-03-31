/*
 * Copyright (C) 2011 Gautier Hattenberger
 * based on ArduIMU driver:
 *   Autoren@ZHAW:  schmiemi
 *                  chaneren
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

#ifndef ArduIMU_H
#define ArduIMU_H

#include "std.h"
#include "math/pprz_algebra_float.h"

extern struct FloatEulers arduimu_eulers;
extern struct FloatRates arduimu_rates;
extern struct FloatVect3 arduimu_accel;

extern float ins_roll_neutral;
extern float ins_pitch_neutral;
extern bool arduimu_calibrate_neutrals;

void ArduIMU_init(void);
void ArduIMU_periodic(void);
void ArduIMU_periodicGPS(void);
void ArduIMU_event(void);

#endif // ArduIMU_H
