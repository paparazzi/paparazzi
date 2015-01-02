#ifndef WT_SERVO_H
#define WT_SERVO_H

#include "std.h"

extern void wt_servo_init(void);
extern void wt_servo_set(uint16_t val);

extern uint16_t wt_servo_motor_power;

#define wt_servo_SetPower(_val) {     \
    wt_servo_motor_power = _val;      \
    wt_servo_set(wt_servo_motor_power);     \
  }



#endif /* WT_SERVO_H */

