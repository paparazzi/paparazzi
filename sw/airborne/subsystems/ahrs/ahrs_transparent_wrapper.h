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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file subsystems/ahrs/ahrs_transparent_wrapper.h
 *
 * Paparazzi specific wrapper to run transparent AHRS
 */

#ifndef AHRS_TRANSPARENT_WRAPPER_H
#define AHRS_TRANSPARENT_WRAPPER_H

#include "subsystems/ahrs/ahrs_transparent.h"

#ifndef PRIMARY_AHRS
#define PRIMARY_AHRS ahrs_transparent
#endif

extern void ahrs_transparent_register(void);

#endif /* AHRS_TRANSPARENT_WRAPPER_H */
