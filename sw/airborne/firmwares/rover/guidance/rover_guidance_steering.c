/* rover_steering_guidance.c */

#define AUTOPILOT_CORE_GUIDANCE_C

// Mandatory headers
#include "firmwares/rover/guidance/rover_guidance_steering.h"

#include "generated/airframe.h"
#include "generated/autopilot_core_guidance.h"

#include "subsystems/datalink/telemetry.h"
#include "subsystems/actuators/actuators_default.h"
#include "subsystems/radio_control.h"
#include "autopilot.h"
#include "navigation.h"
#include "state.h"

#include <stdio.h>


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

  pprz_msg_send_STEERING_ROVER_DATA(trans, dev, AC_ID, &ap_mode, &nav_mode, &rc_t, &rc_r, &cmd_speed, &cmd_steering, &ac_speed, &ac_steering);
}

void rover_guidance_steering_init(void)
{
  // guidance state machine from code generation
  autopilot_core_guidance_init();

  // init debugging telemetry
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STEERING_ROVER_DATA, send_msg);
}

void rover_guidance_steering_periodic(void)
{
  //guidance state machine from code generation 
  #if EXTERN_AP == AP_MODE_NAV 
    autopilot_core_guidance_periodic_task();
  #endif

  // periodic debugging telemetry
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STEERING_ROVER_DATA, send_msg);
}




