#include "dc_mc_power.h"

#include <avr/io.h>

// swicthing freq = CLOCK / RESOLUTION
// 13 bits -> 1953 Hz
#define DC_MC_POWER_RESOLUTION 0x1FFF
// 12 bits -> 3906 Hz
//#define DC_MC_POWER_RESOLUTION 0xFFF
// 11 bits -> 7812 Hz
//#define DC_MC_POWER_RESOLUTION 0x7FF
// 10 bits -> 15625 Hz
//#define DC_MC_POWER_RESOLUTION 0x3FF


void dc_mc_power_init(void) {
  /* OC1A output    */
  DDRB |= _BV(1); 

  /* fast PWM TOP in ICR1 match in OCR1A */
  ICR1 = DC_MC_POWER_RESOLUTION;
  TCCR1A  |= _BV(WGM11) | _BV(COM1A1);
  TCCR1B  |= _BV(WGM12) | _BV(WGM13);
}


void dc_mc_power_set( uint16_t val) {
  OCR1A = val * ((float)DC_MC_POWER_RESOLUTION / 65535.);

}
