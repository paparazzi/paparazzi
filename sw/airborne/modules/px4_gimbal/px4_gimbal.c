/*
 * Copyright (C) Kevin van Hecke
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/px4_gimbal/px4_gimbal.c"
 * @author Kevin van Hecke
 * Control gimbal camera axis through px4 from rc
 */

#include "modules/px4_gimbal/px4_gimbal.h"
#include "modules/radio_control/radio_control.h"

#include "generated/airframe.h" // AC_ID is required
#include "modules/actuators/actuators.h"

#ifndef PX4_GIMBAL_PWM_CHAN
#define PX4_GIMBAL_PWM_CHAN 0
#endif
#ifndef PX4_GIMBAL_RC_CHAN
#define PX4_GIMBAL_RC_CHAN RADIO_AUX2
#endif

#ifndef PX4_GIMBAL_PWM_MIN
#define PX4_GIMBAL_PWM_MIN 900
#endif

#ifndef PX4_GIMBAL_PWM_MAX
#define PX4_GIMBAL_PWM_MAX 1500
#endif


void px4_gimbal_init() {
#ifdef INTER_MCU_AP
   actuators_init();
#endif
}

 void px4_set_gimbal_angle_periodic() {
   //reroute gimbal rc input from fbw to pwm output on ap

   float value =radio_control.values[PX4_GIMBAL_RC_CHAN];

   float range = PX4_GIMBAL_PWM_MAX - PX4_GIMBAL_PWM_MIN;
   value = ((MAX_PPRZ-value) / (float)MAX_PPRZ) * range + PX4_GIMBAL_PWM_MIN;
   if (value < 1) {value=1;} // 0 does not seem to work

   ActuatorPwmSet(PX4_GIMBAL_PWM_CHAN, (int32_t)value);

#ifdef INTER_MCU_AP
   AllActuatorsCommit();
#endif


 }



