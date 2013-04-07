#ifndef SERVOS_NIL_H
#define SERVOS_NIL_H

#define SERVOS_TICS_OF_USEC(s) cpu_ticks_of_usec(s)
#define ChopServo(x,a,b) Chop(x, a, b)
#define Actuator(i) actuators[i]
#define ActuatorsCommit() {}

#endif /* SERVOS_NIL_H */
