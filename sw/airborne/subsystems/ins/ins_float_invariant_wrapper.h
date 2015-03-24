/*
 * Copyright (C) 2015 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/ins/ins_float_invariant_wrapper.h
 *
 * Paparazzi specific wrapper to run INVARIANT filter.
 */

#ifndef INS_FLOAT_INVARIANT_WRAPPER_H
#define INS_FLOAT_INVARIANT_WRAPPER_H

#include "subsystems/ins/ins_float_invariant.h"

#define DefaultInsImpl ins_float_invariant

extern void ins_float_invariant_register(void);

#endif /* INS_FLOAT_INVARIANT_WRAPPER_H */
