
#include <inttypes.h>
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>

#include "avr/std.h"
#include "adc.h"

#define ANALOG_PORT PORTC
#define ANALOG_PORT_DIR DDRC
#define VALIM 7

uint16_t adc_alim;
volatile uint8_t adc_got_val;

void adc_init( void )
{
  /* Ensure that our port is for input with no pull-ups */
  ANALOG_PORT 	&= ~_BV(VALIM);
  ANALOG_PORT_DIR &= ~_BV(VALIM);

  /* Select our external voltage ref, which is tied to Vcc and channel VALIM*/
  ADMUX	= VALIM;

  /* Turn off the analog comparator */
  sbi( ACSR, ACD );

  /* turn on the ADC,  clock/128, interrupts, free running mode and starts conversion */
  ADCSRA = _BV(ADEN) | _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2) | _BV(ADIE) | _BV(ADFR) | _BV(ADSC);
}


SIGNAL( SIG_ADC )
{
  /* Store result */
  adc_alim = ADCW;
  adc_got_val = TRUE;
}
