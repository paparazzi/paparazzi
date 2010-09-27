#include "std.h"
#include "actuators.h"
#include "servos_direct_hw.h"

int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

void actuators_init ( void ) {
  actuators_pwm_arch_init();
}



