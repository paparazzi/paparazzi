/*  $Id$
 *
 * (c) 2003-2005 Pascal Brisset, Antoine Drouin
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


/** Implementation of command.h */

#include <avr/io.h>
#include <avr/signal.h>
#include "servos_4017.h"
#include "actuators.h"
#include "sys_time.h"
#include CONFIG


/* holds the servo pulses width in clock ticks */
uint16_t servo_widths[_4017_NB_CHANNELS];

/*
 * We use the output compare registers to generate our servo pulses.
 * These should be connected to a decade counter that routes the
 * pulses to the appropriate servo.
 *
 * Initialization involves:
 *
 * - Reseting the decade counters
 * - Writing the first pulse width to the counters
 * - Setting output compare to set the clock line by calling servo_enable()
 * - Bringing down the reset lines
 *
 * Ideally, you can use two decade counters to drive 20 servos.
 */
void actuators_init( void ) {
  uint8_t i;
  /* Configure the reset and clock lines as output  */
  _4017_RESET_DDR |= _BV(_4017_RESET_PIN);
  _4017_CLOCK_DDR |= _BV(_4017_CLOCK_PIN);
  /* Reset the decade counter                       */
  sbi( _4017_RESET_PORT, _4017_RESET_PIN );
  /* Lower the clock line                           */
  cbi( _4017_CLOCK_PORT, _4017_CLOCK_PIN );
  /* Set all servos at their midpoints              */
  for( i=0 ; i < _4017_NB_CHANNELS ; i++ )
    servo_widths[i] = SYS_TICS_OF_USEC(1500);
  /* Set servos to go off some long time from now   */
  SERVO_OCR = 32768ul;
  /* Set output compare to toggle the output bits   */
  TCCR1A |=  _BV(SERVO_COM0 );
#ifdef SERVOS_FALLING_EDGE
  /* Starts CLOCK high for the falling edge case    */
  TCCR1A |= _BV(SERVO_FORCE);
#endif
  /* Clear the interrupt flags in case they are set */
  TIFR = _BV(SERVO_FLAG);
  /* Unassert the decade counter reset to start it running */
  cbi( _4017_RESET_PORT, _4017_RESET_PIN );
  /* Enable our output compare interrupts           */
  TIMSK |= _BV(SERVO_ENABLE );
}


/* 
 *  Interrupt routine
 *  
 *  write the next pulse width to OCR register and 
 *  assert the servo signal. It will be cleared by
 *  the following compare match.
 */
SIGNAL( SIG_OUTPUT_COMPARE1A )
{
  static uint8_t servo = 0;
  uint16_t width;

#ifdef SERVOS_FALLING_EDGE
#define RESET_WIDTH SYS_TICS_OF_USEC(1000)
#define FIRST_PULSE_WIDTH SYS_TICS_OF_USEC(100)
/** The clock pin has been initialized high and is toggled down by
the timer.
 Unfortunately it seems that reset does not work on 4017 in this case if it
occurs after the first falling edge. We add two more states at the end of
the sequence:
  - keeping clock low, reset high during 1ms
  - clock high (toggled by the timer), reset down, during 100us (looks like
  the first pulse of a standard RC */
  if (servo == _4017_NB_CHANNELS) {
    sbi( _4017_RESET_PORT, _4017_RESET_PIN );
    /** Start a long 1ms reset, keep clock low */
    SERVO_OCR += RESET_WIDTH;
    servo++;
    return;
  }
  if (servo > _4017_NB_CHANNELS) {
    /** Clear the reset, the clock has been toggled high */
    cbi( _4017_RESET_PORT, _4017_RESET_PIN );
    /** Starts a short pulse-like period */
    SERVO_OCR += FIRST_PULSE_WIDTH;
    servo=0; /** Starts a new sequence next time */
    return;
  }
#else
  if (servo >= _4017_NB_CHANNELS) {
    sbi( _4017_RESET_PORT, _4017_RESET_PIN );
    servo = 0;
    // FIXME: 500 ns required by 4017 reset ???? why does it work without!
    // asm( "nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;" );
    cbi( _4017_RESET_PORT, _4017_RESET_PIN );
  }
#endif
  width = servo_widths[servo];

  SERVO_OCR += width;

  TCCR1A |= _BV(SERVO_FORCE);

  servo++;
}
