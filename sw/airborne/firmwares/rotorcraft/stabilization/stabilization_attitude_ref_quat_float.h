/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_float.h
 *
 * Rotorcraft attitude reference generation.
 * (quaternion float version)
 *
 */

#ifndef STABILIZATION_ATTITUDE_FLOAT_REF_QUAT_FLOAT_H
#define STABILIZATION_ATTITUDE_FLOAT_REF_QUAT_FLOAT_H

#include "stabilization_attitude_ref_float.h"

#ifndef STABILIZATION_ATTITUDE_GAIN_NB
#define STABILIZATION_ATTITUDE_GAIN_NB 1
#endif

#ifndef STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT
#define STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT 0
#endif

void stabilization_attitude_ref_enter(void);
void stabilization_attitude_ref_schedule(uint8_t idx);

#endif /* STABILIZATION_ATTITUDE_FLOAT_REF_QUAT_FLOAT_H */
