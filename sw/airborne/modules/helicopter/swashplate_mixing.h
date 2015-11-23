/*
 * Copyright (C) 2015 C. De Wagter
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/helicopter/swashplate_mixing.h"
 * @author C. De Wagter and Freek van Tienen
 * Helicopter Swashplate Mixing
 */

#ifndef SWASHPLATE_MIXING_H
#define SWASHPLATE_MIXING_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

/* Different type of swashplate mixings */
#define MECH 0
#define H120 1
#define HR120 2

/**
 * MECH (front/right/coll), H120 (front/rightback/leftback), HR120 (back/leftfront/rightfront)
 */
#if SW_MIXING_TYPE == MECH
#define SW_NB 3
#define SW_FRONT   0
#define SW_RIGHT   1
#define SW_COLL    2
#define SW_MIXING_ROLL_COEF   {    0, -1,  0 }
#define SW_MIXING_PITCH_COEF  {    1,  0,  0 }
#define SW_MIXING_COLL_COEF   {    0,  0,  1 }

#elif SW_MIXING_TYPE == H120
#define SW_NB 3
#define SW_FRONT     0
#define SW_RIGHTBACK 1
#define SW_LEFTBACK  2
#define SW_MIXING_ROLL_COEF   {  0, -0.866,  0.866 }
#define SW_MIXING_PITCH_COEF  {  1, -0.5,   -0.5   }
#define SW_MIXING_COLL_COEF   {  1,  1,      1     }

#elif SW_MIXING_TYPE == HR120
#define SW_NB 3
#define SW_BACK       0
#define SW_LEFTFRONT  1
#define SW_RIGHTFRONT 2
#define SW_MIXING_ROLL_COEF   {  0,  0.866, -0.866 }
#define SW_MIXING_PITCH_COEF  { -1,  0.5,    0.5   }
#define SW_MIXING_COLL_COEF   {  1,  1,      1     }
#endif

/* Swashplate mixing structure */
struct swashplate_mixing_t {
  int32_t commands[SW_NB];      ///< The output commands
  int32_t trim[SW_NB];          ///< Trim values for the different actuators
};
extern struct swashplate_mixing_t swashplate_mixing;

/* External used functions */
extern void swashplate_mixing_init(void);
extern void swashplate_mixing_run(pprz_t in_cmd[]);

#endif
