#include "nps_ivy.h"

#include <stdlib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "airframe.h"
#include "pprz_algebra_double.h"
#include "nps_autopilot.h"
#include "nps_fdm.h"


extern void nps_ivy_init(void) {
  const char* agent_name = AIRFRAME_NAME"_NPS";
  const char* ready_msg = AIRFRAME_NAME"_NPS Ready";
  IvyInit(agent_name, ready_msg, NULL, NULL, NULL, NULL);
  //IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  //IvyBindMsg(on_DL_BLOCK, NULL,   "^(\\S*) BLOCK (\\S*) (\\S*)");
  //IvyBindMsg(on_DL_MOVE_WP, NULL, "^(\\S*) MOVE_WP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");
}

extern void nps_ivy_display(void) {


  /*
    IvySendMsg("%d COMMANDS %f %f %f %f",  
    AC_ID,
    autopilot.commands[SERVO_FRONT],
    autopilot.commands[SERVO_BACK], 
    autopilot.commands[SERVO_RIGHT],
    autopilot.commands[SERVO_LEFT]);     
  */
  IvySendMsg("%d BOOZ_SIM_RATE_ATTITUDE %f %f %f %f %f %f",  
	     AC_ID,
	     DegOfRad(fdm.body_rate.p), 
	     DegOfRad(fdm.body_rate.q), 
	     DegOfRad(fdm.body_rate.r),
	     DegOfRad(fdm.ltp_to_body_eulers.phi), 
	     DegOfRad(fdm.ltp_to_body_eulers.theta), 
	     DegOfRad(fdm.ltp_to_body_eulers.psi));
  IvySendMsg("%d BOOZ_SIM_SPEED_POS %f %f %f %f %f %f",  
	     AC_ID,
	     (fdm.ltp_pos.x), 
	     (fdm.ltp_pos.y), 
	     (fdm.ltp_pos.z),
	     (fdm.ltp_ecef_vel.x), 
	     (fdm.ltp_ecef_vel.y), 
	     (fdm.ltp_ecef_vel.z));
}
