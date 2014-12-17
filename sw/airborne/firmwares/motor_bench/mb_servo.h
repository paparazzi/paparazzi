
#ifndef MB_SERVO_H
#define MB_SERVO_H

#include "std.h"

#define MIN_SERVO_US 1000
#define MAX_SERVO_US 2000
#define MIN_SERVO_NS 1000000
#define MAX_SERVO_NS 2000000
#define RANGE_SERVO_US (MAX_SERVO_US - MIN_SERVO_US)

void mb_servo_init(void);
void mb_servo_set_range(uint32_t min_pulse_ns, uint32_t max_pulse_ns);
void mb_servo_set_us(uint32_t duration_us);
//void mb_servo_set_ns(uint32_t duration_ns);
void mb_servo_set(float throttle);
void mb_servo_arm(void);
#endif /* MB_SERVO_H */
