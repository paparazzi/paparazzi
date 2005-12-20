#ifndef PPM_H
#define PPM_H

#include "inttypes.h"
#include "radio.h"

#define PPM_NB_PULSES RADIO_CTL_NB
extern uint16_t ppm_pulses[PPM_NB_PULSES];
extern volatile uint8_t ppm_valid;

void ppm_init ( void );
void TIMER0_ISR ( void ) __attribute__((naked));

#define CLOCK_OF_US(us) ((us)*(PCLK/1000000))

#endif /* PPM_H */
