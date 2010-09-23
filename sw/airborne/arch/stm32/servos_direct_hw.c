#include "std.h"
#include "actuators.h"
#include "servos_direct_hw.h"

int32_t booz_actuators_pwm_values[BOOZ_ACTUATORS_PWM_NB];

void actuators_init ( void ) {
  booz_actuators_pwm_arch_init();
}



