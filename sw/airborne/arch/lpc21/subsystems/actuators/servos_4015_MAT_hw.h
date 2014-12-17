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
 *  \brief Servo driving on MAT0.1 (SERVO_CLOCK_PIN) using TIMER0
 */


#ifndef SERVOS_4015_HW_H
#define SERVOS_4015_HW_H


#include "std.h"
#include "LPC21xx.h"
#include "mcu_periph/sys_time.h"

#include BOARD_CONFIG

#define SERVOS_TICS_OF_USEC(s) cpu_ticks_of_usec(s)
#define ChopServo(x,a,b) Chop(x, a, b)

#define _4015_NB_CHANNELS 8
extern uint16_t servos_values[_4015_NB_CHANNELS];
#define Actuator4015Set(_i, _v) { servos_values[_i] = SERVOS_TICS_OF_USEC(_v); }

extern void actuators_4015_init(void);
#define Actuators4015Commit() {}
#define Actuators4015Init() actuators_4015_init()
extern uint8_t servos_4015_idx;
extern uint32_t servos_delay;

#define SERVO_REFRESH_TICS SERVOS_TICS_OF_USEC(20000)

#define ACTUATORS_IT TIR_MR1I
#define Servos4015Mat_ISR() {       \
    if (servos_4015_idx == 0) {       \
      servos_delay = SERVO_REFRESH_TICS;      \
      SetBit(IO1CLR, SERVO_DATA_PIN);     \
    }             \
    if (servos_4015_idx < _4015_NB_CHANNELS ) {   \
      T0MR1 += servos_values[servos_4015_idx];    \
      servos_delay -= servos_values[servos_4015_idx]; \
      servos_4015_idx++;          \
    } else if (servos_4015_idx == _4015_NB_CHANNELS) {  \
      SetBit(IO1SET, SERVO_RESET_PIN);      \
      servos_4015_idx++;          \
      T0MR1 += servos_delay/2;        \
    } else  {           \
      SetBit(IO1SET, SERVO_DATA_PIN);     \
      SetBit(IO1CLR, SERVO_RESET_PIN);      \
      servos_4015_idx = 0;        \
      T0MR1 += servos_delay/2;        \
    }             \
    /* lower clock pin */         \
    T0EMR &= ~TEMR_EM1;         \
  }

#endif /* SERVOS_4015_HW_H */
