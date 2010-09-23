#include "ant_servo.h"

#include <avr/io.h>

#include CONFIG

#define PRESCALER 8
#define TICKS_OF_US(nb_us) (((nb_us) * (float)(CLOCK / PRESCALER)-1))
#define SERVO_REFRESH 25000

void ant_servo_init( void ) {
  /* set PB5 and PB6 (OC1A and OC1B ) as output */
  SetBit (DDRB, 5);
  SetBit (DDRB, 6);
  /* set timer1 in fast PWM mode, with TOP defined by ICR3 , prescaled to 8 */
  TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1);
  TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS11);
  ICR1=TICKS_OF_US(SERVO_REFRESH);
  ant_servo_set(NEUTRAL_SERVO, NEUTRAL_SERVO);
}

void ant_servo_set ( uint16_t value1_us, uint16_t value2_us) {
/* code pour regler la valeur en ms a l'etat haut du signal PWM */
  Bound(value1_us, MIN_SERVO, MAX_SERVO);
  OCR1A = TICKS_OF_US(value1_us);
  Bound(value2_us, MIN_SERVO, MAX_SERVO);
  OCR1B = TICKS_OF_US(value2_us);
}
