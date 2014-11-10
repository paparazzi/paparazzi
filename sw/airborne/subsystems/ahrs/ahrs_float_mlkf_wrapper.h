/*
 * Copyright (C) 2014 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/ahrs/ahrs_float_mlkf_wrapper.h
 *
 * Paparazzi specific wrapper to run MLKF filter.
 */

#ifndef AHRS_FLOAT_MLKF_WRAPPER_H
#define AHRS_FLOAT_MLKF_WRAPPER_H

#include "subsystems/ahrs/ahrs_float_mlkf.h"

#define DefaultAhrsImpl ahrs_mlkf

extern void ahrs_mlkf_register(void);

#endif /* AHRS_FLOAT_MLKF_WRAPPER_H */
