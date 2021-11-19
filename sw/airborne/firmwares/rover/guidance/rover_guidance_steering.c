/*
 * Copyright (C) 2021 Jesús Bautista <jesusbautistavillar@gmail.com> 
 *                    Hector García <noeth3r@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#define AUTOPILOT_CORE_GUIDANCE_C

/** Mandatory dependencies header **/
#include "firmwares/rover/guidance/rover_guidance_steering.h"

#include "generated/airframe.h"

#include "subsystems/datalink/telemetry.h"
#include "subsystems/actuators/actuators_default.h"
#include "subsystems/radio_control.h"
#include "autopilot.h"
#include "navigation.h"
#include "state.h"

#include <math.h>
#include <stdio.h>

// Static guidance variables
static float SR_Ke = 10000.0; // TODO: configurable parameter in guidance_control struct 

// Guidance control main struct
rover_ctrl guidance_control;


/** Send RS guidance telemetry messages **/
static void send_msg(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t ap_mode  = autopilot_get_mode();

  pprz_msg_send_STEERING_ROVER_DATA(trans, dev, AC_ID, 
                                    &ap_mode, &nav.mode, 
                                    &commands[COMMAND_THROTTLE], &commands[COMMAND_STEERING], 
                                    &actuators[SERVO_MOTOR_THROTTLE_IDX], &actuators[SERVO_MOTOR_STEERING_IDX], 
                                    &guidance_control.cmd.delta,
                                    &guidance_control.cmd.speed,
                                    &guidance_control.gvf_omega, 
                                    &guidance_control.state_speed);
}

bool rover_guidance_steering_set_delta(float delta){
  guidance_control.cmd.delta = delta;
  return true;
}


/** INIT function**/
void rover_guidance_steering_init(void)
{
  // Debugging telemetry init
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STEERING_ROVER_DATA, send_msg);

  guidance_control.cmd.delta = 0.0;
  guidance_control.cmd.speed = 0.0;
  guidance_control.throttle = 0.0;
}


/** PERIODIC function **/
void rover_guidance_steering_periodic(void)
{ 
  guidance_control.state_speed = stateGetHorizontalSpeedNorm_f();

  // speed is bounded to avoid GPS noise while driving at small velocity
  float delta = 0.0;
  float speed = BoundSpeed(guidance_control.state_speed); 
  float omega = guidance_control.gvf_omega; //GVF give us this omega

  // ASSISTED guidance .....................................................
  if (autopilot_get_mode() == AP_MODE_ASSISTED) {
    if (fabs(omega)>0.0) {
      delta = DegOfRad(-atanf(omega * DRIVE_SHAFT_DISTANCE / speed));
    }

    guidance_control.cmd.delta = BoundDelta(delta);
  }

  // NAV guidance ...........................................................
  else if (autopilot_get_mode() == AP_MODE_NAV) {
    // In NAV mode, we can set cmd.delta and cmd.speed from GCS settings panel
    guidance_control.cmd.delta = BoundDelta(guidance_control.cmd.delta);

    // Control speed signal (not implemented yet, this is just for NPS tests)
    float error = guidance_control.cmd.speed - guidance_control.state_speed;
    guidance_control.throttle = BoundThrottle(SR_Ke * error); // Simple control model...
  } 

  // FAILSAFE values ........................................................
  else {
    guidance_control.cmd.delta = 0.0;
    guidance_control.cmd.speed = 0.0;
  }

  // periodic steering_rover telemetry
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STEERING_ROVER_DATA, send_msg);
}




