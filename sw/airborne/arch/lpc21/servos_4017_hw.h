#ifndef SERVOS_4017_HW_H
#define SERVOS_4017_HW_H

#include <inttypes.h>
#include "std.h"

#include "LPC21xx.h"
#include "mcu_periph/sys_time.h"

#include BOARD_CONFIG

#define SERVOS_TICS_OF_USEC(s) CPU_TICKS_OF_USEC(s)
#define ChopServo(x,a,b) Chop(x, a, b)

#if defined NB_CHANNELS
#define _4017_NB_CHANNELS Chop(NB_CHANNELS,0,10)
#else
#define _4017_NB_CHANNELS 10
#endif

extern uint16_t servos_values[_4017_NB_CHANNELS];
#define Actuator(i) servos_values[i]

#define ActuatorsCommit() {}

extern uint8_t servos_4017_idx;

#define ACTUATORS_IT TIR_MR1I

#ifndef SERVOS_4017_CLOCK_FALLING
#define SERVOS_4017_ISR() {					\
    if (servos_4017_idx >= _4017_NB_CHANNELS) {			\
      SetBit(IO1SET, SERVO_RESET_PIN);				\
      servos_4017_idx = 0;					\
      SetBit(IO1CLR, SERVO_RESET_PIN);				\
    }								\
    								\
    /* request clock high on next match */			\
    T0MR1 += servos_values[servos_4017_idx];			\
    /* lower clock pin */					\
    T0EMR &= ~TEMR_EM1;						\
    servos_4017_idx++;						\
  }
#else /* SERVOS_4017_CLOCK_FALLING */

#define SERVOS_4017_RESET_WIDTH SERVOS_TICS_OF_USEC(1000)
#define SERVOS_4017_FIRST_PULSE_WIDTH SERVOS_TICS_OF_USEC(100)

#define SERVOS_4017_ISR() {					\
   if (servos_4017_idx == _4017_NB_CHANNELS) {			\
      SetBit(IO1SET, SERVO_RESET_PIN);				\
      /* Start a long 1ms reset, keep clock low */		\
      T0MR1 += SERVOS_4017_RESET_WIDTH;				\
      servos_4017_idx++;					\
      T0EMR &= ~TEMR_EM1;					\
    }								\
    else if (servos_4017_idx > _4017_NB_CHANNELS) {		\
      /* Clear the reset*/					\
      SetBit(IO1CLR,SERVO_RESET_PIN);				\
      /* assert clock       */					\
      T0EMR |= TEMR_EM1;					\
      /* Starts a short pulse-like period */			\
      T0MR1 += SERVOS_4017_FIRST_PULSE_WIDTH;			\
      servos_4017_idx=0; /** Starts a new sequence next time */	\
    }								\
    else {							\
      /* request next match */					\
      T0MR1 += servos_values[servos_4017_idx];			\
      /* clock low if not last one, last is done with reset */	\
      if (servos_4017_idx != _4017_NB_CHANNELS-1) {		\
        /* raise clock pin */					\
        T0EMR |= TEMR_EM1;					\
      }								\
      servos_4017_idx++;					\
    }								\
  }
#endif /* SERVOS_4017_CLOCK_ON_RESET */

#endif /* SERVOS_4017_HW_H */
