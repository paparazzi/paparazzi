/*
 * Paparazzi AVR adc functions
 *  
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin, Michel Gorraz
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

//// ADC3 MVSUP
//// ADC6 MVSERVO


#if (__GNUC__ == 3)
#include <avr/signal.h>
#endif
#include <avr/interrupt.h>
#include <avr/io.h>
#include "adc.h"
#include "std.h"


static struct adc_buf* buffers[NB_ADC];

void adc_buf_channel(uint8_t adc_channel, struct adc_buf* s, uint8_t av_nb_sample) {
  buffers[adc_channel] = s;
  s->av_nb_sample = av_nb_sample;
}


/************************************************************************/
#if defined (__AVR_ATmega8__)


#define VOLTAGE_TIME	0x07
#define ANALOG_PORT	PORTC
#define ANALOG_PORT_DIR	DDRC


#ifdef IMU_ANALOG
#define ANALOG_VREF _BV(REFS0)
#else
#define ANALOG_VREF _BV(REFS0) | _BV(REFS1)
#endif



/**
 * Called when the voltage conversion is finished
 * 
 *  8.913kHz on mega128 16MHz 1kHz/channel ??
*/
SIGNAL( SIG_ADC )
{
  uint8_t adc_input	= ADMUX & 0x7;
  struct adc_buf* buf = buffers[adc_input];
  uint16_t adc_value = ADCW;

  if (buf) {
    uint8_t new_head = buf->head + 1;
    if (new_head >= buf->av_nb_sample) new_head = 0;
    buf->sum -= buf->values[new_head];
    buf->values[new_head] = adc_value;
    buf->sum += adc_value;
    buf->head = new_head;   
  }

  /* Find the next input */
  adc_input++;
  if (adc_input == 4)
    adc_input = 6; // ADC 4 and 5 for i2c
  if( adc_input >= 8 ) {
    adc_input = 0;
#ifdef CTL_BRD_V1_2
    adc_input = 1; // WARNING ADC0 is for rservo driver reset on v1.2.0
#endif /* CTL_BRD_V1_2 */
  }
  /* Select it */
  ADMUX = adc_input | ANALOG_VREF;
  /* Restart the conversion */
  sbi( ADCSR, ADSC );
}

#endif /* (__AVR_ATmega8__) */




/************************************************************************/
#if defined  (__AVR_ATmega128__)

#define VOLTAGE_TIME	0x07
#define ANALOG_PORT	PORTF
#define ANALOG_PORT_DIR	DDRF


#define ANALOG_VREF _BV(REFS0)


/**
 * Called when the voltage conversion is finished
 * 
 *  8.913kHz on mega128 16MHz 1kHz/channel ??
*/
SIGNAL( SIG_ADC )
{
  uint8_t adc_input	= ADMUX & 0x7;
  struct adc_buf* buf = buffers[adc_input];
  uint16_t adc_value = ADCW;

  if (buf) {
    uint8_t new_head = buf->head + 1;
    if (new_head >= buf->av_nb_sample) new_head = 0;
    buf->sum -= buf->values[new_head];
    buf->values[new_head] = adc_value;
    buf->sum += adc_value;
    buf->head = new_head;   
  }

  /* Find the next input */
  adc_input++;
  if( adc_input >= 8 )
    adc_input = 0;
  /* Select it */
  ADMUX = adc_input | ANALOG_VREF;
  /* Restart the conversion */
  sbi( ADCSR, ADSC );
}
#endif /* (__AVR_ATmega128__) */



void adc_init( void ) {
  uint8_t i;
  /* Ensure that our port is for input with no pull-ups */
  ANALOG_PORT = 0x00;
  ANALOG_PORT_DIR = 0x00;

  /* Select our external voltage ref */
  ADMUX	= ANALOG_VREF;

  /* Select out clock, turn on the ADC interrupt and start conversion */
  ADCSRA = 0
    | VOLTAGE_TIME
    | _BV(ADEN )
    | _BV(ADIE )
    | _BV(ADSC );

  /* Init to 0 (usefull ?) */
  for(i = 0; i < NB_ADC; i++)
    buffers[i] = (struct adc_buf*)0;
}

