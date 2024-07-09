/*
 *
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
 * @file arch/chibios/modules/actuators/actuators_t4_arch.h
 * Actuator interface for T4 driver
 */
#ifndef ACTUATORS_T4_ARCH_H
#define ACTUATORS_T4_ARCH_H

#include "std.h"

// servo 1~10(0~10) + esc 1~4 (10~13)
#ifndef ACTUATORS_T4_NB
#define ACTUATORS_T4_NB 14
#endif

extern int32_t actuators_t4_values[ACTUATORS_T4_NB];

extern void actuators_t4_commit(void);

#define ActuatorT4Set(_i, _v) { actuators_t4_values[_i] = _v; }
#define ActuatorsT4Commit  actuators_t4_commit

#endif /* ACTUATORS_T4_ARCH_H */
