#ifndef ANT_SERVO_H
#define ANT_SERVO_H

#include "std.h"

#define MAX_SERVO     2000
#define NEUTRAL_SERVO 1500
#define MIN_SERVO     1000

void ant_servo_init( void );
void ant_servo_set ( uint16_t value1_us, uint16_t value2_us);

#endif /* ANT_SERVO_H */
