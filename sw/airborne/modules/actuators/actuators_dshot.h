/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/actuators/actuators_dshot.h"
 * @author Gautier Hattenberger
 * Driver for DSHOT speed controller protocol
 */

#ifndef ACTUATORS_DSHOT_H
#define ACTUATORS_DSHOT_H

#include "std.h"
#include "modules/actuators/actuators_dshot_arch.h"

/** In normal DSHOT, first 48 values are special commands
 *  this offset allow to use 0 as the no-throttle command
 *  This should not be changed unless you know what you are doing
 */
#ifndef ACTUATORS_DSHOT_OFFSET
#define ACTUATORS_DSHOT_OFFSET 48
#endif

/** Maxnum number of DSHOT commands
 *  This should be large enough for max applications:
 *  8 motors +1 in case motor count starts at 1 and not 0
 */
#ifndef ACTUATORS_DSHOT_NB
#define ACTUATORS_DSHOT_NB (8+1)
#endif

extern uint16_t actuators_dshot_values[ACTUATORS_DSHOT_NB];

/** Arch dependent init
 */
extern void actuators_dshot_arch_init(void);
extern void actuators_dshot_arch_commit(void);

/* Actuator macros */
#define ActuatorDShotSet(_i, _v) { \
 if (_v == 0) { actuators_dshot_values[_i] = 0; } \
 else { actuators_dshot_values[_i] = _v + ACTUATORS_DSHOT_OFFSET; } \
}
#define ActuatorsDShotInit() actuators_dshot_arch_init()
#define ActuatorsDShotCommit() actuators_dshot_arch_commit()

#endif

