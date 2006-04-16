#ifndef PPM_HW_H
#define PPM_HW_H

#include "LPC21xx.h"
#include CONFIG

static inline void ppm_init ( void ) {
   /* select pin for capture */
  PPM_PINSEL |= PPM_PINSEL_VAL << PPM_PINSEL_BIT;
  /* enable capture 0.2 on rising edge + trigger interrupt */
  T0CCR = TCCR_CR2_R | TCCR_CR2_I;

  ppm_valid = FALSE;
}

#define PPM_NB_CHANNEL PPM_NB_PULSES


#define PPM_ISR() {						\
   static uint8_t state = PPM_NB_CHANNEL;			\
    static uint32_t last;					\
								\
    uint32_t now = T0CR2;					\
    uint32_t length = now - last;				\
    last = now;							\
								\
    if (state == PPM_NB_CHANNEL) {				\
      if (length > SYS_TICS_OF_USEC(PPM_SYNC_MIN_LEN) &&	\
	  length < SYS_TICS_OF_USEC(PPM_SYNC_MAX_LEN)) {	\
	state = 0;						\
      }								\
    }								\
    else {							\
      if (length > SYS_TICS_OF_USEC(PPM_DATA_MIN_LEN) &&	\
	  length < SYS_TICS_OF_USEC(PPM_DATA_MAX_LEN)) {	\
	ppm_pulses[state] = length;				\
	state++;						\
	if (state == PPM_NB_CHANNEL) {				\
	  ppm_valid = TRUE;					\
	}							\
      }								\
      else							\
	state = PPM_NB_CHANNEL;					\
    }								\
}


#endif /* PPM_HW_H */
