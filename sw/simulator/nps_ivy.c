#include "nps_ivy.h"
#include "nps_autopilot.h"
#include "pprz_algebra_double.h"

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>


extern void ivy_transport_init(void) {
  IvyInit("BoozSim", "BoozSim READY", NULL, NULL, NULL, NULL);
  //IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  //IvyBindMsg(on_DL_BLOCK, NULL, "^(\\S*) BLOCK (\\S*) (\\S*)");
  //IvyBindMsg(on_DL_MOVE_WP, NULL, "^(\\S*) MOVE_WP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");
}

extern void nps_ivy_display(void) {

  DoubleEulers eulers;
  
  DOUBLE_EULERS_OF_QUAT(eulers, fdm.ltp_to_body_quat);

  IvySendMsg("%d COMMANDS %f %f %f %f",  
	     AC_ID,
	     autopilot.commands[SERVO_FRONT],
	     autopilot.commands[SERVO_BACK], 
	     autopilot.commands[SERVO_RIGHT],
	     autopilot.commands[SERVO_LEFT]);     
  IvySendMsg("%d BOOZ_SIM_RATE_ATTITUDE %f %f %f %f %f %f",  
	     AC_ID,
	     DegOfRad(bfm.ang_rate_body->ve[AXIS_X]), 
	     DegOfRad(bfm.ang_rate_body->ve[AXIS_Y]), 
	     DegOfRad(bfm.ang_rate_body->ve[AXIS_Z]),
	     DegOfRad(bfm.eulers->ve[AXIS_X]), 
	     DegOfRad(bfm.eulers->ve[AXIS_Y]), 
	     DegOfRad(bfm.eulers->ve[AXIS_Z]));
  IvySendMsg("%d BOOZ_SIM_SPEED_POS %f %f %f %f %f %f",  
	     AC_ID,
	     (bfm.speed_ltp->ve[AXIS_X]), 
	     (bfm.speed_ltp->ve[AXIS_Y]), 
	     (bfm.speed_ltp->ve[AXIS_Z]),
	     (bfm.pos_ltp->ve[AXIS_X]), 
	     (bfm.pos_ltp->ve[AXIS_Y]), 
	     (bfm.pos_ltp->ve[AXIS_Z]));
}
