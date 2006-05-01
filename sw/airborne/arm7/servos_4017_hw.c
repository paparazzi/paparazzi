#include "actuators.h"

#include "paparazzi.h"
#include "airframe.h"

uint8_t servos_4017_idx;

#define START_TIMEOUT 0xFFFF;

void actuators_init ( void ) {
  /* select clock pin as MAT0.1 output */
  IO0DIR |= _BV(SERVO_CLOCK_PIN);
  SERVO_CLOCK_PINSEL |= SERVO_CLOCK_PINSEL_VAL << SERVO_CLOCK_PINSEL_BIT;

  /* select reset pin as GPIO output */
  IO1DIR |= _BV(SERVO_RESET_PIN);
  PINSEL2 &= ~(_BV(3)); /* P1.25-16 are used as GPIO */
  //  SERVO_RESET_PINSEL |= SERVO_RESET_PINSEL_VAL << SERVO_RESET_PINSEL_BIT;
  /* assert RESET */
  IO1SET = _BV(SERVO_RESET_PIN);

  /* enable match 1 interrupt */
  T0MCR |= TMCR_MR1_I;
  
#ifndef SERVOS_4017_CLOCK_FALLING
  /* lower clock         */
  T0EMR &= ~TEMR_EM1;	
  /* set high on match 1 */
  T0EMR |= TEMR_EMC11;
#else
  /* assert clock       */
  T0EMR |= TEMR_EM1;	
  /* set low on match 1 */
  T0EMR |= TEMR_EMC10;
#endif
  /* set first pulse in a while */
  T0MR1 = START_TIMEOUT;
  servos_4017_idx = _4017_NB_CHANNELS;
  /* Set all servos at their midpoints */
  /* compulsory for unaffected servos  */
  uint8_t i;
  for( i=0 ; i < _4017_NB_CHANNELS ; i++ )
    servos_values[i] = SERVOS_TICS_OF_USEC(1500);
}


uint16_t servos_values[_4017_NB_CHANNELS];

