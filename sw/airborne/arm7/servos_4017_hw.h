#ifndef SERVOS_4017_HW_H
#define SERVOS_4017_HW_H

#include <inttypes.h>
#include "std.h"

#include "LPC21xx.h"
#include "sys_time.h"

#include CONFIG

#define SERVOS_TICS_OF_USEC(s) SYS_TICS_OF_USEC(s)
#define ChopServo(x,a,b) Chop(x, a, b)

#define _4017_NB_CHANNELS 10
extern uint16_t servos_values[_4017_NB_CHANNELS];
#define Actuator(i) servos_values[i]

extern uint8_t servos_4017_idx;


#ifndef SERVOS_4017_CLOCK_FALLING    
#define SERVOS_4017_ISR() {					\
    if (servos_4017_idx >= _4017_NB_CHANNELS) {			\
      IO1SET = _BV(SERVO_RESET_PIN);				\
      servos_4017_idx = 0;					\
      IO1CLR = _BV(SERVO_RESET_PIN);				\
    }								\
    								\
    /* request clock high on next match */			\
    T0MR1 += servos_values[servos_4017_idx];			\
    /* lower clock pin */					\
    T0EMR &= ~TEMR_EM1;						\
    servos_4017_idx++;						\
  }
#else /* SERVOS_4017_CLOCK_ON_RESET */
#define SERVOS_4017_RESET_WIDTH SERVOS_TICS_OF_USEC(1000)
#define SERVOS_4017_FIRST_PULSE_WIDTH SERVOS_TICS_OF_USEC(100)
#define SERVOS_4017_ISR() {					\
  if (servos_4017_idx == _4017_NB_CHANNELS) {			\
      IO1SET = _BV(SERVO_RESET_PIN);				\
      /* Start a long 1ms reset, keep clock low */		\
      T0MR1 += SERVOS_4017_RESET_WIDTH;				\
      servos_4017_idx++;					\
    }								\
    else if (servos_4017_idx > _4017_NB_CHANNELS) {		\
      /* Clear the reset*/					\
      IO1CLR = _BV(SERVO_RESET_PIN);				\
      /* assert clock       */					\
      T0EMR |= TEMR_EM1;					\
      /* Starts a short pulse-like period */			\
      T0MR1 += SERVOS_4017_FIRST_PULSE_WIDTH;			\
      servos_4017_idx=0; /** Starts a new sequence next time */	\
    }								\
    else {							\
      /* request clock low on next match */			\
      T0MR1 += servos_values[servos_4017_idx];			\
      /* raise clock pin */					\
      T0EMR |= TEMR_EM1;					\
      servos_4017_idx++;					\
    }								\
  }
#endif /* SERVOS_4017_CLOCK_ON_RESET */	

#endif /* SERVOS_4017_HW_H */
