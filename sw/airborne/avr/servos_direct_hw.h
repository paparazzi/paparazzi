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
#define ChopServo(x, min, max) Chop(x, min, max)

#define SERVO_REG_0 OCR3A
#define SERVO_REG_1 OCR3B
#define SERVO_REG_2 OCR3C
#define COMMAND_(i) SERVO_REG_ ## i
#define Actuator(i) COMMAND_(i)



#endif /* SERVOS_DIRECT_HW_H */
