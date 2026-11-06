/*
 * Copyright (C) 2010 The Paparazzi Team
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

#ifndef ACTUATORS_dualpwm_H
#define ACTUATORS_dualpwm_H

#include "modules/actuators/actuators_driver.h"

#include "modules/actuators/actuators_dualpwm_arch.h"

/** Arch dependent init file.
 * implemented in arch files
 */
extern void actuators_dualpwm_arch_init(void);
extern void actuators_dualpwm_set(uint8_t idx, int16_t value);

static inline void actuators_dualpwm_driver_init(uint8_t driver_id)
{
  actuators_register_driver(driver_id, (struct actuator_driver) {
    .set = actuators_dualpwm_set,
    .commit = actuators_noop_commit,
  });
  actuators_dualpwm_arch_init();
}

#define ActuatorsDualpwmInit actuators_dualpwm_driver_init

#endif /* ACTUATORS_PWM_H */
