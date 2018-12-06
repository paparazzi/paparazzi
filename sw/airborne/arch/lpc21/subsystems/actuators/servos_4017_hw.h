/*
 * Copyright (C) 2012 ENAC
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

#ifndef SERVOS_4017_HW_H
#define SERVOS_4017_HW_H

#include <inttypes.h>
#include "std.h"

#include "LPC21xx.h"
#include "mcu_periph/sys_time.h"

#include BOARD_CONFIG

#define SERVOS_TICS_OF_USEC(s) cpu_ticks_of_usec(s)
#define ClipServo(x,a,b) Clip(x, a, b)

#if defined NB_CHANNELS
#define _4017_NB_CHANNELS Clip(NB_CHANNELS,0,10)
#else
#define _4017_NB_CHANNELS 10
#endif

extern uint16_t servos_values[_4017_NB_CHANNELS];
#define Actuator4017Set(_i, _v) { servos_values[_i] = SERVOS_TICS_OF_USEC(_v); }

extern void actuators_4017_init(void);
#define Actuators4017Commit() {}
#define Actuators4017Init() actuators_4017_init()

extern uint8_t servos_4017_idx;

#define ACTUATORS_IT TIR_MR1I

#ifndef SERVOS_4017_CLOCK_FALLING
#define SERVOS_4017_ISR() {         \
    if (servos_4017_idx >= _4017_NB_CHANNELS) {     \
      SetBit(IO1SET, SERVO_RESET_PIN);        \
      servos_4017_idx = 0;          \
      SetBit(IO1CLR, SERVO_RESET_PIN);        \
    }               \
    \
    /* request clock high on next match */      \
    T0MR1 += servos_values[servos_4017_idx];      \
    /* lower clock pin */         \
    T0EMR &= ~TEMR_EM1;           \
    servos_4017_idx++;            \
  }
#else /* SERVOS_4017_CLOCK_FALLING */

#define SERVOS_4017_RESET_WIDTH SERVOS_TICS_OF_USEC(1000)
#define SERVOS_4017_FIRST_PULSE_WIDTH SERVOS_TICS_OF_USEC(100)

#define SERVOS_4017_ISR() {         \
    if (servos_4017_idx == _4017_NB_CHANNELS) {      \
      SetBit(IO1SET, SERVO_RESET_PIN);        \
      /* Start a long 1ms reset, keep clock low */    \
      T0MR1 += SERVOS_4017_RESET_WIDTH;       \
      servos_4017_idx++;          \
      T0EMR &= ~TEMR_EM1;         \
    }               \
    else if (servos_4017_idx > _4017_NB_CHANNELS) {   \
      /* Clear the reset*/          \
      SetBit(IO1CLR,SERVO_RESET_PIN);       \
      /* assert clock       */          \
      T0EMR |= TEMR_EM1;          \
      /* Starts a short pulse-like period */      \
      T0MR1 += SERVOS_4017_FIRST_PULSE_WIDTH;     \
      servos_4017_idx=0; /** Starts a new sequence next time */ \
    }               \
    else {              \
      /* request next match */          \
      T0MR1 += servos_values[servos_4017_idx];      \
      /* clock low if not last one, last is done with reset */  \
      if (servos_4017_idx != _4017_NB_CHANNELS-1) {   \
        /* raise clock pin */         \
        T0EMR |= TEMR_EM1;          \
      }               \
      servos_4017_idx++;          \
    }               \
  }
#endif /* SERVOS_4017_CLOCK_ON_RESET */

#endif /* SERVOS_4017_HW_H */
