#ifndef PPM_H
#define PPM_H

#include "types.h"
#include "radio.h"

#define PPM_NB_CHANNEL RADIO_CTL_NB
extern uint16_t ppm_pulses[PPM_NB_CHANNEL];
extern volatile uint8_t ppm_valid;

void ppm_init ( void );
void TIMER0_ISR ( void ) __attribute__((naked));

#define CLOCK_OF_US(us) ((us)*(PCLK/1000000))

#endif /* PPM_H */
