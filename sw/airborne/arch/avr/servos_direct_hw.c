/* Implementation of command.h */

/* 
   3 servos on OC3A OC3B OC3C using Timer3 prescaled at 8
*/

#include "servos_direct_hw.h"
#include "std.h"
#include "actuators.h"
#include "airframe.h"

void actuators_init ( void ) {
 /* OC3A, OC3B, OC3C outputs    */
  DDRE |= _BV(3) | _BV(4) | _BV(5);
  /* set timer3 in fast PWM mode, with TOP defined by ICR3 , prescaled to 8 */
  TCCR3A = _BV(WGM31) | _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1);
  TCCR3B = _BV(WGM32) | _BV(WGM33) | _BV(CS31);
  /* set timer3 rollover */
  ICR3 = TIMER3_TOP;
  /* Set all servos at their midpoints */
  Actuator(0) = SERVOS_TICS_OF_USEC(1500);
  Actuator(1) = SERVOS_TICS_OF_USEC(1500);
  Actuator(2) = SERVOS_TICS_OF_USEC(1500);
}

