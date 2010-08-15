#include "actuators/booz_actuators_pwm.h"

int32_t booz_actuators_pwm_values[BOOZ_ACTUATORS_PWM_NB];

void booz_actuators_init(void) {
  booz_actuators_pwm_arch_init();
}
