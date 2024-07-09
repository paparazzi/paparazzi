/*
 * Copyright (C) 2024 Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef ACTUATORS_T4X_H
#define ACTUATORS_T4X_H

#include "actuators_t4.h"

/** Stub file needed per t4 interface because of generator */
extern int16_t actuators_t4x_values[SERVOS_T4X_NB];

#if USE_NPS
#define ActuatorsT4xInit() {}
#define ActuatorT4xSet(_i, _v) {}
#define ActuatorsT4xCommit()  {}
#else
#define ActuatorsT4xInit() actuators_t4_init(&t4x)
#define ActuatorT4xSet(_i, _v) { actuators_t4x_values[_i] = _v; }
#define ActuatorsT4xCommit()  actuators_t4_commit(&t4x, actuators_t4x_values, SERVOS_T4X_NB)
#endif

#endif /* ACTUATORS_T4X_H */