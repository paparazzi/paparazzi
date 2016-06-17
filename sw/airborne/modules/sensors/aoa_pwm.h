/*
* Copyright (C) 2015 Jean-François Erdelyi, Gautier Hattenberger
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
* @file "modules/sensors/aoa_pwm.h"
* @author Jean-François Erdelyi
* @brief Angle of Attack sensor on PWM
*
* SENSOR, example : US DIGITAL MA3-P12-125-B
*/

#ifndef AOA_PWM_H
#define AOA_PWM_H

#include "std.h"

struct Aoa_Pwm {
  uint32_t raw; ///< raw PWM value
  float angle;  ///< Angle of attack in radians
  float offset; ///< Angle of attack offset in radians
  float sens;   ///< sensitiviy, i.e. scale to conver raw to angle

  /** Filtering value [0-1]
   * 0: no filtering
   * 1: output is a constant value
   */
  float filter;
};

extern struct Aoa_Pwm aoa_pwm;

extern void aoa_pwm_init(void);
extern void aoa_pwm_update(void);

#endif /* AOA_PWM_H */

