/*
 * Copyright (C) 2015 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ahrs/ahrs_float_invariant_wrapper.h
 *
 * Paparazzi specific wrapper to run INVARIANT ahrs filter.
 */

#ifndef AHRS_FLOAT_INVARIANT_WRAPPER_H
#define AHRS_FLOAT_INVARIANT_WRAPPER_H

#include "modules/ahrs/ahrs_float_invariant.h"

#ifndef PRIMARY_AHRS
#define PRIMARY_AHRS ahrs_float_invariant
#endif

extern void ahrs_float_invariant_register(void);

#endif /* AHRS_FLOAT_INVARIANT_WRAPPER_H */
