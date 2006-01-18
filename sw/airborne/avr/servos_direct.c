/* Implementation of command.h */

/* 
   3 servos on OC3A OC3B OC3C using Timer3 prescaled at 8
*/
#include <avr/io.h>
#include "std.h"
#include "command.h"
#include "airframe.h"
#include CONFIG

const pprz_t failsafe_values[COMMANDS_NB] = COMMANDS_FAILSAFE;

#define SERVO_REG_0 OCR3A
#define SERVO_REG_1 OCR3B
#define SERVO_REG_2 OCR3C
#define COMMAND_(i) SERVO_REG_ ## i
#define COMMAND(i) COMMAND_(i)

/* servo refresh rate in HZ */
#define SERVO_REFRESH_RATE 50
/* timer3 prescaler         */
#define TIMER3_PRESCALER 8
#define TIMER3_TOP (CLOCK*1e6/SERVO_REFRESH_RATE/TIMER3_PRESCALER)
#define SERVOS_TICS_OF_USEC(s) ((s)*CLOCK/TIMER3_PRESCALER)


void command_set(const pprz_t values[]) {
  CommandsSet(values); /*Generated from airframe.xml */
}
