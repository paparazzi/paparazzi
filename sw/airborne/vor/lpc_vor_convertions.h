#ifndef LPC_VOR_CONVERTIONS_H
#define LPC_VOR_CONVERTIONS_H

#include "LPC21xx.h"

#include "std.h"


extern volatile uint16_t vor_adc_sample;
extern volatile bool_t vor_adc_sample_available;

// DAC on P0.25
#define VorDacInit() { \
    /* turn on DAC pins */			\
    PINSEL1 &=  1 << 19;			\
    PINSEL1 |= ~(1 << 18);			\
  }


#define VorDacSet(a) {				\
    DACR = a << 6;				\
  }


extern void vor_adc_init( void );

#endif /* LPC_VOR_CONVERTIONS_H */

