#include "csc_servos.h"

#include "LPC21xx.h"
#include "std.h"
#include "sys_time.h"
#include "actuators.h"
#include "airframe.h"
#include ACTUATORS

#define CSC_SERVOS_NB 4

static uint32_t csc_servos_rng[] = {SYS_TICS_OF_USEC(SERVO_S1_MAX-SERVO_S1_MIN),
				    SYS_TICS_OF_USEC(SERVO_S2_MAX-SERVO_S2_MIN),
				    SYS_TICS_OF_USEC(SERVO_S3_MAX-SERVO_S3_MIN),
				    SYS_TICS_OF_USEC(SERVO_S4_MAX-SERVO_S4_MIN)};
static uint32_t csc_servos_min[] = {SYS_TICS_OF_USEC(SERVO_S1_MIN),
				    SYS_TICS_OF_USEC(SERVO_S2_MIN),
				    SYS_TICS_OF_USEC(SERVO_S3_MIN),
				    SYS_TICS_OF_USEC(SERVO_S4_MIN)};


void csc_servos_init(void)
{
  actuators_init();
}

/* val == 0 literally mean off, otherwise, val 1-255 are continuous angle */
void csc_servo_normalized_set(uint8_t id, uint16_t val)
{
  if(id > 3) return;
  if(val == 0) csc_servo_set(id,0);
  else{
    uint32_t ticks = csc_servos_rng[id]*(val-1);
  
    csc_servo_set(id,ticks/((1<<16)-2) + csc_servos_min[id]);
  }
}


void csc_servos_commit()
{
  ActuatorsCommit();
}

void csc_servo_set(uint8_t id, uint32_t val)
{
  if(id > 3) return;

#ifdef CSC_MOTORS_I2C
  Actuator(id) = val;
#else
  switch(id){
  case 0: Actuator(0) = val; break;
  case 1: Actuator(5) = val; break;
  case 2: Actuator(4) = val; break;
  case 3: Actuator(3) = val; break;
  }
#endif
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

  csc_servos_commit();
}


