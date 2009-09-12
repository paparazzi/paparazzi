#include "booz_drop.h"
#include "booz2_pwm_hw.h"


bool_t booz_drop_ball;
int16_t booz_drop_servo;

#define DROP_SERVO_OPEN 1700
#define DROP_SERVO_CLOSED 900

void booz_drop_init(void) {
  booz_drop_ball = FALSE;
  booz_drop_periodic();
}

void booz_drop_periodic(void) {
  if (booz_drop_ball == TRUE)
    booz_drop_servo = DROP_SERVO_OPEN;
  else
    booz_drop_servo = DROP_SERVO_CLOSED;
  Booz2SetPwmValue(booz_drop_servo);
}
