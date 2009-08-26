#include "nps_ivy.h"

#include <stdlib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "airframe.h"
#include "math/pprz_algebra_double.h"
#include "nps_autopilot.h"
#include "nps_fdm.h"

#include NPS_SENSORS_PARAMS

static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]);

void nps_ivy_init(void) {
  const char* agent_name = AIRFRAME_NAME"_NPS";
  const char* ready_msg = AIRFRAME_NAME"_NPS Ready";
  IvyInit(agent_name, ready_msg, NULL, NULL, NULL, NULL);
  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  //IvyBindMsg(on_DL_BLOCK, NULL,   "^(\\S*) BLOCK (\\S*) (\\S*)");
  //IvyBindMsg(on_DL_MOVE_WP, NULL, "^(\\S*) MOVE_WP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");
}

#include "settings.h"
#include "dl_protocol.h"
#include "downlink.h"
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)),
			  void *user_data __attribute__ ((unused)),
			  int argc __attribute__ ((unused)), char *argv[]) {
  uint8_t index = atoi(argv[2]);
  float value = atof(argv[3]);
  DlSetting(index, value);
  DOWNLINK_SEND_DL_VALUE(DOWNLINK_TRANSPORT, &index, &value);
  printf("setting %d %f\n", index, value);
}

void nps_ivy_display(void) {


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
	     DegOfRad(fdm.body_ecef_rotvel.p),
	     DegOfRad(fdm.body_ecef_rotvel.q),
	     DegOfRad(fdm.body_ecef_rotvel.r),
	     DegOfRad(fdm.ltp_to_body_eulers.phi),
	     DegOfRad(fdm.ltp_to_body_eulers.theta),
	     DegOfRad(fdm.ltp_to_body_eulers.psi));
  IvySendMsg("%d BOOZ_SIM_SPEED_POS %f %f %f %f %f %f %f %f %f",
	     AC_ID,
	     (fdm.ltpprz_ecef_accel.x),
	     (fdm.ltpprz_ecef_accel.y),
	     (fdm.ltpprz_ecef_accel.z),
	     (fdm.ltpprz_ecef_vel.x),
	     (fdm.ltpprz_ecef_vel.y),
	     (fdm.ltpprz_ecef_vel.z),
	     (fdm.ltpprz_pos.x),
	     (fdm.ltpprz_pos.y),
	     (fdm.ltpprz_pos.z));
  IvySendMsg("%d BOOZ_SIM_GYRO_BIAS %f %f %f",
         AC_ID,
         DegOfRad(RATE_FLOAT_OF_BFP(sensors.gyro.bias_random_walk_value.x)+sensors.gyro.bias_initial.x),
         DegOfRad(RATE_FLOAT_OF_BFP(sensors.gyro.bias_random_walk_value.y)+sensors.gyro.bias_initial.y),
         DegOfRad(RATE_FLOAT_OF_BFP(sensors.gyro.bias_random_walk_value.z)+sensors.gyro.bias_initial.z));

  /* transform magnetic field to body frame */
  struct DoubleVect3 h_body;
  FLOAT_QUAT_VMULT(h_body, fdm.ltp_to_body_quat, fdm.ltp_h);

  IvySendMsg("%d BOOZ_SIM_SENSORS_SCALED %f %f %f %f %f %f",
         AC_ID,
         ((sensors.accel.value.x - sensors.accel.neutral.x)/NPS_ACCEL_SENSITIVITY_XX),
         ((sensors.accel.value.y - sensors.accel.neutral.y)/NPS_ACCEL_SENSITIVITY_YY),
         ((sensors.accel.value.z - sensors.accel.neutral.z)/NPS_ACCEL_SENSITIVITY_ZZ),
         h_body.x,
         h_body.y,
         h_body.z);
}
