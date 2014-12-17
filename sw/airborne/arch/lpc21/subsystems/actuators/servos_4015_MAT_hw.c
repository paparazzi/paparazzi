/*
 * Copyright (C) 2006  Antoine Drouin
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

/** \file servos_4015_MAT_hw.c
 *  \brief Servo driving MAT0.1 (SERVO_CLOCK_PIN) using TIMER0
 */


#include "subsystems/actuators.h"
#include "paparazzi.h"
#include "generated/airframe.h"

uint8_t servos_4015_idx;
uint32_t servos_delay;

#define START_TIMEOUT 0xFFFF;

void actuators_4015_init(void)
{
  /* select clock pin as MAT0.1 output */
  IO0DIR |= _BV(SERVO_CLOCK_PIN);
  SERVO_CLOCK_PINSEL |= SERVO_CLOCK_PINSEL_VAL << SERVO_CLOCK_PINSEL_BIT;

  /* select reset pin as GPIO output */
  IO1DIR |= _BV(SERVO_RESET_PIN);
  PINSEL2 &= ~(_BV(3)); /* P1.25-16 are used as GPIO */
  /* assert RESET */
  IO1SET = _BV(SERVO_RESET_PIN);

  /* DATA pin output */
  IO1DIR |= _BV(SERVO_DATA_PIN);

  /* enable match 1 interrupt */
  T0MCR |= TMCR_MR1_I;

  /* lower clock         */
  T0EMR &= ~TEMR_EM1;
  /* set high on match 1 */
  T0EMR |= TEMR_EMC1_2;

  /* set first pulse in a while */
  T0MR1 = START_TIMEOUT;
  servos_4015_idx = _4015_NB_CHANNELS;
  /* Set all servos at their midpoints */
  /* compulsory for unaffected servos  */
  uint8_t i;
  for (i = 0 ; i < _4015_NB_CHANNELS ; i++) {
    servos_values[i] = SERVOS_TICS_OF_USEC(1500);
  }

  servos_delay = SERVO_REFRESH_TICS;
}


uint16_t servos_values[_4015_NB_CHANNELS];

