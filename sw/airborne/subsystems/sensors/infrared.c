/*
 * Copyright (C) 2003-2010  The Paparazzi Team
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

/** \file subsystems/sensors/infrared.c
 *  \brief common infrared
 */

#include "subsystems/sensors/infrared.h"
#include "generated/airframe.h"

struct Infrared infrared;

/** \brief Initialisation of \a ir structure
 */
void infrared_struct_init(void)
{
  infrared.roll_neutral = IR_ROLL_NEUTRAL_DEFAULT;
  infrared.pitch_neutral = IR_PITCH_NEUTRAL_DEFAULT;

  infrared.correction_left = IR_CORRECTION_LEFT;
  infrared.correction_right = IR_CORRECTION_RIGHT;
  infrared.correction_up = IR_CORRECTION_UP;
  infrared.correction_down = IR_CORRECTION_DOWN;

  infrared.lateral_correction = IR_LATERAL_CORRECTION;
  infrared.longitudinal_correction = IR_LONGITUDINAL_CORRECTION;
  infrared.vertical_correction = IR_VERTICAL_CORRECTION;

}


