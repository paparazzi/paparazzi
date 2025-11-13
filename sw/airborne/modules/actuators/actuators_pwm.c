/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/actuators/actuators_pwm.c"
 * @author Gautier Hattenberger
 */

#include "modules/actuators/actuators_pwm.h"

int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

void actuators_pwm_set(uint8_t idx, int16_t value)
{
  if (idx < ACTUATORS_PWM_NB) {
    actuators_pwm_values[idx] = value;
  }
}


