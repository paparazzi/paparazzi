/*  $Id$
 *
 * Copied from autopilot (autopilot.sf.net) thanx alot Trammell
 * (c) 2002 Trammell Hudson <hudson@rotomotion.com>
 * (c) 2003 Pascal Brisset, Antoine Drouin
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


#include <avr/io.h>
#include <avr/signal.h>
#include "servo.h"
#include "link_autopilot.h"

#include "airframe.h"

#include "uart.h"


/*
 * Paparazzi boards have one 4017 servo driver.
 * It is driven by OCR1A (PB1) with reset on PORTD5.
 */
#define _4017_NB_CHANNELS 10

#ifdef CTL_BRD_V1_2
#define _4017_RESET_PORT        PORTC
#define _4017_RESET_DDR         DDRC
#define _4017_RESET_PIN         0
#endif /* CTL_BRD_V1_2 */

#ifdef CTL_BRD_V1_2_1
#define _4017_RESET_PORT        PORTD
#define _4017_RESET_DDR         DDRD
#define _4017_RESET_PIN         7
#endif /* CTL_BRD_V1_2 */

#define _4017_CLOCK_PORT        PORTB
#define _4017_CLOCK_DDR         DDRB
#define _4017_CLOCK_PIN         PB1

#define SERVO_OCR		OCR1A
#define SERVO_ENABLE		OCIE1A
#define SERVO_FLAG		OCF1A
#define SERVO_FORCE		FOC1A
#define SERVO_COM0		COM1A0
#define SERVO_COM1		COM1A1

/* Following macro is required since the compiler does not solve at
   compile-time indexation in a known array with a known index */
#define SERVO_NEUTRAL_(i) SERVOS_NEUTRALS_ ## i
#define SERVO_NEUTRAL(i) (SERVO_NEUTRAL_(i)*CLOCK)

#define SERVO_NEUTRAL_I(i) (((int[])SERVOS_NEUTRALS[i])*CLOCK)
#define SERVO_MIN_I(i) (((int[])SERVOS_MINS[i])*CLOCK)

#define SERVO_MIN (SERVO_MIN_US*CLOCK)
#define SERVO_MAX (SERVO_MAX_US*CLOCK)
#define ChopServo(x) ((x) < SERVO_MIN ? SERVO_MIN : ((x) > SERVO_MAX ? SERVO_MAX : (x)))

/* holds the servo pulses width in clock ticks */
static uint16_t servo_widths[_4017_NB_CHANNELS];

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
void
servo_init( void )
{
  uint8_t			i;

  /* Configure the reset and clock lines */
  _4017_RESET_DDR |= _BV(_4017_RESET_PIN);
  _4017_CLOCK_DDR |= _BV(_4017_CLOCK_PIN);

  /* Reset the decade counter */
  sbi( _4017_RESET_PORT, _4017_RESET_PIN );

  /* Lower the regular servo line */
  cbi( _4017_CLOCK_PORT, _4017_CLOCK_PIN );

  /* Set all servos at their midpoints */
  for( i=0 ; i < _4017_NB_CHANNELS ; i++ )
    //    servo_widths[i] = SERVO_MIN;
    servo_widths[i] = (SERVO_MIN+SERVO_MAX)/2;
	
  /* Set servos to go off some long time from now */
  SERVO_OCR	= 32768ul;

  /*
   * Configure output compare to toggle the output bits.
   */
  TCCR1A |=  _BV(SERVO_COM0 );
	
  /* Clear the interrupt flags in case they are set */
  TIFR = _BV(SERVO_FLAG);

  /* Unassert the decade counter reset to start it running */
  cbi( _4017_RESET_PORT, _4017_RESET_PIN );

  /* Enable our output compare interrupts */
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

  if (servo >= _4017_NB_CHANNELS) {
    sbi( _4017_RESET_PORT, _4017_RESET_PIN );
    servo = 0;
    // FIXME: 500 ns required by 4017 reset ???? why does it work without!
    // asm( "nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;" );
    cbi( _4017_RESET_PORT, _4017_RESET_PIN );
  }

  width = servo_widths[servo];

  SERVO_OCR += width;

  TCCR1A |= _BV(SERVO_FORCE);

  servo++;
}

void servo_set_one(uint8_t servo, uint16_t value_us) {
  servo_widths[servo] = ChopServo(CLOCK*value_us);
}

void 
servo_transmit(void) {
  uint8_t servo;
  uart_transmit((uint8_t)0); uart_transmit((uint8_t)0);

  for(servo = 0; servo < _4017_NB_CHANNELS; servo++) {
    uart_transmit((uint8_t)(servo_widths[servo] >> 8));
    uart_transmit((uint8_t)(servo_widths[servo] & 0xff));
  }
  uart_transmit((uint8_t)'\n');
}


/*
 *
 * defines how servos react to radio control or autopilot channels
 *
 */

void servo_set(const pprz_t values[]) {
  ServoSet(values); /*Generated from airframe.xml */
}
