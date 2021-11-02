/* rover_steering_guidance.c */

#define AUTOPILOT_CORE_GUIDANCE_C

/** Mandatory dependencies header **/
#include "firmwares/rover/guidance/rover_guidance_steering.h"

#include "generated/airframe.h"
#include "generated/autopilot_core_guidance.h"

#include "subsystems/datalink/telemetry.h"
#include "subsystems/actuators/actuators_default.h"
#include "subsystems/radio_control.h"
#include "autopilot.h"
#include "navigation.h"
#include "state.h"

#include <math.h>
#include <stdio.h>

// Control
rover_ctrl guidance_control;

/** Send DEBUG Telemetry messages **/
static void send_msg(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t ap_mode  = autopilot_get_mode();
  uint8_t nav_mode = nav.mode;
  int16_t rc_t = radio_control.values[RADIO_THROTTLE];
  int16_t rc_r = radio_control.values[RADIO_ROLL]; 
  int16_t cmd_speed    = commands[COMMAND_SPEED];
  int16_t cmd_steering = commands[COMMAND_STEERING];
  int16_t ac_speed     = actuators[SERVO_MOTOR_THROTTLE_IDX];
  int16_t ac_steering  = actuators[SERVO_MOTOR_STEERING_IDX];

  pprz_msg_send_STEERING_ROVER_DATA(trans, dev, AC_ID, &ap_mode, &nav_mode, &rc_t, &rc_r, 
                                    &cmd_speed, &cmd_steering, &ac_speed, &ac_steering, 
                                    &guidance_control.omega, &guidance_control.delta,
                                    &guidance_control.speedNorm);
}

bool rover_guidance_steering_set_delta(float delta){
  guidance_control.delta = delta;
  return true;
}

/** INIT function**/
void rover_guidance_steering_init(void)
{
  // Guidance state machine init
  autopilot_core_guidance_init();

  // Debugging telemetry init
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STEERING_ROVER_DATA, send_msg);

  guidance_control.delta = 0.0;
}

/** PERIODIC function **/
void rover_guidance_steering_periodic(void)
{ 
  // Definitions
  float delta;
  float R = guidance_control.r;
  float Kv = 0.01; //TODO: struct in .h

  // Check GPS state values
  guidance_control.speedNorm = stateGetHorizontalSpeedNorm_f();
  guidance_control.speedDir = stateGetHorizontalSpeedDir_f();

  // Bounding Speed Norm
  float speed = BoundSpeed(guidance_control.speedNorm);

  // ASSISTED guidance
  if (autopilot_get_mode() == AP_MODE_ASSISTED) {
    delta = 0.0;
    // delta = DRIVE_SHAFT_DISTANCE/R + speed**2/R; //TODO: check L/R sign
    if (fabs(guidance_control.omega)>0.0) {
      delta += -atanf(guidance_control.omega * DRIVE_SHAFT_DISTANCE / speed);
    }
    delta *= 180/PI;

    guidance_control.delta = BoundDelta(delta);
  }

  // NAV guidance
  else if (autopilot_get_mode() == AP_MODE_NAV) {
    autopilot_core_guidance_periodic_task();
    guidance_control.delta = 0.0;
  } 

  // FAILSAFE values
  else {
    guidance_control.delta = 0.0;
  }

  // periodic steering_rover telemetry
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STEERING_ROVER_DATA, send_msg);
}




