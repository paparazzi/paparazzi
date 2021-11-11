/*
 * (c) 2003-2005 Pascal Brisset, Antoine Drouin
 * (c) 2012 Gautier Hattenberger
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

/** @file modules/actuators/actuators.h
 *  Hardware independent API for actuators (servos, motor controllers).
 *
 */
#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "paparazzi.h"
#include "std.h"

/*
 * Defines SetActuatorsFromCommands() macro
 * Defines ACTUATORS_NB to 0 if no servo
 * Include servos drivers
 */
#include "generated/airframe.h"

#if ACTUATORS_NB

extern void actuators_init(void);

extern uint32_t actuators_delay_time;
extern bool   actuators_delay_done;

/** Actuators array.
 * Temporary storage (for debugging purpose, downlinked via telemetry)
 * */
extern int16_t actuators[ACTUATORS_NB];

/** PPRZ command to each actuator
 * Can be used to directly control actuators from the control algorithm
 * if the command_laws are set up appropriately in the airframe file
 */
extern int16_t actuators_pprz[ACTUATORS_NB];

/** Set actuators.
 * @param _n actuators name as given in airframe file, servos section
 * @param _v new actuator's value
 */
#define _ActuatorSet(_n, _v) Set_ ## _n ## _Servo(_v)
#define ActuatorSet(_n, _v) _ActuatorSet(_n, _v)

#endif /* ACTUATORS_NB */


#endif /* ACTUATORS_H */
