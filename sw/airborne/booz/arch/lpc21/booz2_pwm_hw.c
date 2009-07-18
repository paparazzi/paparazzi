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

#include "booz2_pwm_hw.h"

#define PWM_PERIOD SYS_TICS_OF_USEC(20000)
#define PWM_DUTY SYS_TICS_OF_USEC(1500)

void booz2_pwm_init_hw( void ) {

  /* start PWM5 */
  /* select P0.21 as PWM5 */
  PWM_PINSEL |=  PWM_PINSEL_VAL << PWM_PINSEL_BIT;
  /* select pwm period */
  PWMMR0 = PWM_PERIOD;
  /* select pwm value to 50% at init (1500 us) */
  PWMMR5 = PWM_DUTY;
  /* commit values */
  PWMLER = PWMLER_LATCH0 | PWMLER_LATCH5;
  /* enable counter and pwm mode */
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE;
  /* enable PWM5 */
  PWMPCR = PWMPCR_ENA5;

}


