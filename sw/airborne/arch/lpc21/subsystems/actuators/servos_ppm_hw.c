/*
 * Copyright (C) 2008  Mark Griffin
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

/**
 * @file arch/lpc21/subsystems/actuators/servos_ppm_hw.c
 *
 * Efficient driving of MAT0.1 (SERVO_CLOCK_PIN) using TIMER0 to produce PPM
 * for a R/C receiver which has a microcontroller to drive the servos
 * (not a 4015 or 4017 decade counter chip).
 */

#include "subsystems/actuators.h"
#include "paparazzi.h"
#include "generated/airframe.h"

uint8_t servos_PPM_idx;
uint8_t ppm_pulse;
uint32_t servos_delay;

#define START_TIMEOUT 0xFFFF;

void actuators_ppm_init(void)
{
  /* select ppm output pin as MAT0.1 output */
  SERVO_CLOCK_PINSEL |= SERVO_CLOCK_PINSEL_VAL << SERVO_CLOCK_PINSEL_BIT;

  /* enable match 1 interrupt */
  T0MCR |= TMCR_MR1_I;

  /* lower clock         */
  T0EMR &= ~TEMR_EM1;
  /* set high on match 1 */
  T0EMR |= TEMR_EMC1_2;

  /* set first pulse in a while */
  T0MR1 = START_TIMEOUT;
  servos_PPM_idx = _PPM_NB_CHANNELS;
  /* Set all servos to their midpoints */
  /* compulsory for unused servos  */
  uint8_t i;
  for (i = 0 ; i < _PPM_NB_CHANNELS ; i++) {
    servos_values[i] = SERVOS_TICS_OF_USEC(1500);
  }

  servos_delay = SERVO_REFRESH_TICS;
}
uint16_t servos_values[_PPM_NB_CHANNELS];

