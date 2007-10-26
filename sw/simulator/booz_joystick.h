#ifndef BOOZ_JOYSTICK_H
#define BOOZ_JOYSTICK_H

#define JS_ROLL     0
#define JS_PITCH    1
#define JS_MODE     2
#define JS_YAW      5
#define JS_THROTTLE 6
#define JS_NB_AXIS  7

extern void booz_joystick_init(const char* device);

extern double booz_joystick_value[JS_NB_AXIS];





#endif /* BOOZ_JOYSTICK_H */
