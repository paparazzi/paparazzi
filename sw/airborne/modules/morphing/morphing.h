/*
 * Copyright (C) 2021 Murat Bronz
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

/** @file modules/morphing/morphing.h
 *
 * Geometry morphing of a vehicle with the help of servo(s).
 */

#ifndef MORPHING_H
#define MORPHING_H

#include "generated/airframe.h"

extern int32_t servo_1_val;
extern int32_t servo_2_val;

extern float morph_command_1;
extern float morph_command_2;
extern float morph_dance_period;

extern bool morph_do_dance;

void morphing_init(void);
void morphing_periodic(void);

#endif /* MORPHING_H */
