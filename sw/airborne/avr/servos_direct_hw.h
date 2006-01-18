#ifndef SERVOS_DIRECT_HW_H
#define SERVOS_DIRECT_HW_H

#include <avr/io.h>
#include CONFIG

/* servo refresh rate in HZ */
#define SERVO_REFRESH_RATE 50
/* timer3 prescaler         */
#define TIMER3_PRESCALER 8
#define TIMER3_TOP (CLOCK*1e6/SERVO_REFRESH_RATE/TIMER3_PRESCALER)
#define SERVOS_TICS_OF_USEC(s) ((s)*CLOCK/TIMER3_PRESCALER)

#define SERVO_REG_0 OCR3A
#define SERVO_REG_1 OCR3B
#define SERVO_REG_2 OCR3C
#define COMMAND_(i) SERVO_REG_ ## i
#define COMMAND(i) COMMAND_(i)

static inline void command_init ( void ) {
 /* OC3A, OC3B, OC3C outputs    */
  DDRE |= _BV(3) | _BV(4) | _BV(5);
  /* set timer3 in fast PWM mode, with TOP defined by ICR3 , prescaled to 8 */
  TCCR3A = _BV(WGM31) | _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1);
  TCCR3B = _BV(WGM32) | _BV(WGM33) | _BV(CS31);
  /* set timer3 rollover */
  ICR3 = TIMER3_TOP;
  /* Set all servos at their midpoints */
  COMMAND(0) = SERVOS_TICS_OF_USEC(1500);
  COMMAND(1) = SERVOS_TICS_OF_USEC(1500);
  COMMAND(2) = SERVOS_TICS_OF_USEC(1500);
  /* Load failsafe values              */
  command_set(failsafe_values);
}



#endif /* SERVOS_DIRECT_HW_H */
