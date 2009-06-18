#include "csc_servos.h"

#include "LPC21xx.h"
#include "std.h"
#include "sys_time.h"
#include "actuators.h"
#include ACTUATORS

#define CSC_SERVOS_NB 4
#define SERVOS_PERIOD (SYS_TICS_OF_SEC((1./250.))); /* 250 Hz */

void csc_servos_init(void)
{
  actuators_init();
}

void csc_servos_set(int32_t* val)
{
#ifdef CSC_MOTORS_I2C
  Actuator(0) = val[0];
  Actuator(1) = val[1];
  Actuator(2) = val[2];
  Actuator(3) = val[3];
#else
  Actuator(0) = val[0];
  Actuator(5) = val[1];
  Actuator(4) = val[2];
  Actuator(3) = val[3];
#endif

  ActuatorsCommit();
}

