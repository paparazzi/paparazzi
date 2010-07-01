/*
 * $Id: $
 *  
 * Copyright (C) 2007  ENAC
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
 *
 */

#ifndef BOOZ2_PWM_HW_H
#define BOOZ2_PWM_HW_H

#include "std.h"
#include "sys_time.h"

extern void booz2_pwm_init_hw(void);

// Default PWM is PWM0
#define Booz2SetPwmValue(_v) Booz2SetPwm0Value(_v)

#define Booz2SetPwm0Value(_v) { \
  PWMMR5 = SYS_TICS_OF_USEC(_v); \
  PWMLER = PWMLER_LATCH5; \
}

#define Booz2SetPwm1Value(_v) { \
  PWMMR2 = SYS_TICS_OF_USEC(_v); \
  PWMLER = PWMLER_LATCH2; \
}

#endif /* BOOZ2_PWM_HW_H */
