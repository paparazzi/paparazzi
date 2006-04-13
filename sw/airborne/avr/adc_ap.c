/*
 * $Id$
 *  
 * Originally from autopilot (autopilot.sf.net) thanx alot Trammell
 *
 * Copyright (C) 2002 Trammell Hudson <hudson@rotomotion.com>
 * Copyright (C) 2003-2005 Pascal Brisset, Antoine Drouin
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


#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "airframe.h"
#include "std.h"
#include "adc_ap.h"


/*************************************************************************
 *
 *  Analog to digital conversion code.
 *
 * We allow interrupts during the 2048 usec windows.  If we run the
 * ADC clock faster than Clk/64 we have too much overhead servicing
 * the interrupts from it and end up with servo jitter.
 *
 * For now we've slowed the clock to Clk/128 because it lets us
 * be lazy in the interrupt routine.
 */
#define VOLTAGE_TIME	0x07
#define ANALOG_PORT	PORTF
#define ANALOG_PORT_DIR	DDRF


#define ANALOG_VREF _BV(REFS0)

static struct adc_buf* buffers[NB_ADC];

void adc_buf_channel(uint8_t adc_channel, struct adc_buf* s, uint8_t av_nb_sample) {
  buffers[adc_channel] = s;
  s->av_nb_sample = av_nb_sample;
}

void 
adc_init( void )
{
  uint8_t i;
  /* Ensure that our port is for input with no pull-ups */
  ANALOG_PORT 	= 0x00;
  ANALOG_PORT_DIR	= 0x00;

  /* Select our external voltage ref, which is tied to Vcc */
  ADMUX		= ANALOG_VREF;

  /* Turn off the analog comparator */
  sbi( ACSR, ACD );

  /* Select out clock, turn on the ADC interrupt and start conversion */
  ADCSR		= 0
    | VOLTAGE_TIME
    | ( 1 << ADEN )
    | ( 1 << ADIE )
    | ( 1 << ADSC );

  /* Init to 0 (usefull ?) */
  for(i = 0; i < NB_ADC; i++)
    buffers[i] = (struct adc_buf*)0;
}

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
