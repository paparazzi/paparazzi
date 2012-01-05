/*
 * $Id: $
 *
 * Copyright (C) 2011  ENAC
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

#include "modules/core/booz_pwm_arch.h"

#define PWM_PERIOD CPU_TICKS_OF_USEC(20000)
#define PWM_DUTY CPU_TICKS_OF_USEC(1500)

void booz_pwm_init_arch( void ) {

  /* start PWM5 */
  /* select P0.21 as PWM5 */
  PWM0_PINSEL |=  PWM0_PINSEL_VAL << PWM0_PINSEL_BIT;
  /* select pwm period */
  PWMMR0 = PWM_PERIOD;
  /* select pwm value to 50% at init (1500 us) */
  PWMMR5 = PWM_DUTY;
  /* commit values */
  PWMLER = PWMLER_LATCH0 | PWMLER_LATCH5;
  /* prescle timer to match TIMER 0 (15MHz) */
  PWMPR = T0_PCLK_DIV - 1;
  /* enable counter and pwm mode */
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE;
  /* enable PWM5 */
  PWMPCR = PWMPCR_ENA5;

#ifdef USE_PWM1
  /* start PWM2 */
  PWM1_PINSEL |=  PWM1_PINSEL_VAL << PWM1_PINSEL_BIT;
  /* select pwm value to 50% at init (1500 us) */
  PWMMR2 = PWM_DUTY;
  /* commit values */
  PWMLER = PWMLER_LATCH2;
  /* enable PWM2 */
  PWMPCR |= PWMPCR_ENA2;
#endif


}
