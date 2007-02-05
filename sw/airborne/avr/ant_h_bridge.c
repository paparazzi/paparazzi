#include "ant_h_bridge.h"

#include <avr/io.h>

#include CONFIG

#define HB_DDR  DDRE
#define HB_PORT PORTE
/* OC3A */
#define HB_IN1_PIN 3
/* OC3C */
#define HB_IN2_PIN 5
/* ENABLE */
#define HB_EN_PIN  7

#define HB_REFRESH 500


void ant_h_bridge_init( void ) {
  /* set PE1, PE3 and PE5 (OC3A, OC3C and PD0) as output */
  SetBit (HB_DDR,HB_IN1_PIN); /* input 1 */
  SetBit (HB_DDR,HB_IN2_PIN); /* input 2 */
  SetBit (HB_DDR,HB_EN_PIN);  /* enable  */


  /* disable motor */
  //  ClearBit(HB_PORT, HB_EN_PIN);
  SetBit(HB_PORT, HB_EN_PIN);

  /* set timer3 in fast PWM mode, with TOP defined by ICR3 , prescaled to 8 */

  TCCR3A = _BV(WGM31) | _BV(COM3A1) | _BV(COM3C1);
  TCCR3B = _BV(WGM32) | _BV(WGM33) | _BV(CS31);
  ICR3 = HB_REFRESH;
  ant_h_bridge_set(0);
  //  ant_h_bridge_set(HB_REFRESH/5);
  //  SetBit(HB_PORT, HB_IN1_PIN);
  //  ClearBit(HB_PORT, HB_IN2_PIN);

}

void ant_h_bridge_set ( int16_t value) {
  if (value > 0) {
    OCR3A = value;
    OCR3C = 0;
  }
  else {
    OCR3A = 0;
    OCR3C = -value;
  }
}
