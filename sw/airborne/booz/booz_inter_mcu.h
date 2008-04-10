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

#ifndef BOOZ_INTER_MCU_H
#define BOOZ_INTER_MCU_H

#include "std.h"
#include "6dof.h"

/* angles are transmitted as int16 : PI -> 16384 */
/* rates are transmitted as int16 : 1 PI s-1 -> 16384 */
#define ANGLE_PI 0X3FFF
#define RATE_PI_S 0X3FFF

struct booz_inter_mcu_state {
  int16_t r_rates [AXIS_NB];
  int16_t f_rates [AXIS_NB];
  int16_t f_eulers[AXIS_NB];
  int16_t pad0;
  float   pos[AXIS_NB];
  float   speed[AXIS_NB];
  uint8_t status;
  uint8_t pad1;
  uint16_t crc;
};

extern struct booz_inter_mcu_state inter_mcu_state;

#ifdef BOOZ_FILTER_MCU
extern void inter_mcu_fill_state(void);
#endif

#endif /* BOOZ_INTER_MCU_H */
