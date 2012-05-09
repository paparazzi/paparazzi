#include "nps_ivy.h"

#include <stdlib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "generated/airframe.h"
#include "math/pprz_algebra_double.h"
#include "nps_autopilot.h"
#include "nps_fdm.h"
#include "nps_sensors.h"
#include "firmwares/rotorcraft/navigation.h"

#include NPS_SENSORS_PARAMS


/* Datalink Ivy functions */
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)),
                          void *user_data __attribute__ ((unused)),
                          int argc __attribute__ ((unused)), char *argv[]);

static void on_DL_GET_SETTING(IvyClientPtr app __attribute__ ((unused)),
                              void *user_data __attribute__ ((unused)),
                              int argc __attribute__ ((unused)), char *argv[]);

static void on_DL_PING(IvyClientPtr app __attribute__ ((unused)),
                       void *user_data __attribute__ ((unused)),
                       int argc __attribute__ ((unused)), char *argv[]);

static void on_DL_BLOCK(IvyClientPtr app __attribute__ ((unused)),
                        void *user_data __attribute__ ((unused)),
                        int argc __attribute__ ((unused)), char *argv[]);

static void on_DL_MOVE_WP(IvyClientPtr app __attribute__ ((unused)),
                          void *user_data __attribute__ ((unused)),
                          int argc __attribute__ ((unused)), char *argv[]);

void nps_ivy_init(void) {
  const char* agent_name = AIRFRAME_NAME"_NPS";
  const char* ready_msg = AIRFRAME_NAME"_NPS Ready";
  IvyInit(agent_name, ready_msg, NULL, NULL, NULL, NULL);
  IvyBindMsg(on_DL_PING, NULL, "^(\\S*) DL_PING");
  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_GET_SETTING, NULL, "^(\\S*) DL_GET_SETTING (\\S*) (\\S*)");
  IvyBindMsg(on_DL_BLOCK, NULL,   "^(\\S*) BLOCK (\\S*) (\\S*)");
  IvyBindMsg(on_DL_MOVE_WP, NULL, "^(\\S*) MOVE_WP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");
}

//TODO use datalink parsing from booz or fw instead of doing it here explicitly
//FIXME currently parsed correctly for booz only


#include "generated/settings.h"
#include "dl_protocol.h"
#include "subsystems/datalink/downlink.h"
static void on_DL_SETTING(IvyClientPtr app __attribute__ ((unused)),
                          void *user_data __attribute__ ((unused)),
                          int argc __attribute__ ((unused)), char *argv[]) {
  uint8_t index = atoi(argv[2]);
  float value = atof(argv[3]);
  DlSetting(index, value);
  DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &index, &value);
  printf("setting %d %f\n", index, value);
}

static void on_DL_GET_SETTING(IvyClientPtr app __attribute__ ((unused)),
                              void *user_data __attribute__ ((unused)),
                              int argc __attribute__ ((unused)), char *argv[]) {
  uint8_t index = atoi(argv[2]);
  float value = settings_get_value(index);
  DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &index, &value);
  printf("get setting %d %f\n", index, value);
}

static void on_DL_PING(IvyClientPtr app __attribute__ ((unused)),
                       void *user_data __attribute__ ((unused)),
                       int argc __attribute__ ((unused)), char *argv[] __attribute__ ((unused))) {
  DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
}

static void on_DL_BLOCK(IvyClientPtr app __attribute__ ((unused)),
                        void *user_data __attribute__ ((unused)),
                        int argc __attribute__ ((unused)), char *argv[]){
  int block = atoi(argv[1]);
  nav_goto_block(block);
  printf("goto block %d\n", block);
}

static void on_DL_MOVE_WP(IvyClientPtr app __attribute__ ((unused)),
                          void *user_data __attribute__ ((unused)),
                          int argc __attribute__ ((unused)), char *argv[]) {
  uint8_t wp_id = atoi(argv[1]);

  struct LlaCoor_i lla;
  struct EnuCoor_i enu;
  lla.lat = INT32_RAD_OF_DEG(atoi(argv[3]));
  lla.lon = INT32_RAD_OF_DEG(atoi(argv[4]));
  lla.alt = atoi(argv[5]) - ins_ltp_def.hmsl + ins_ltp_def.lla.alt;
  enu_of_lla_point_i(&enu,&ins_ltp_def,&lla);
  enu.x = POS_BFP_OF_REAL(enu.x)/100;
  enu.y = POS_BFP_OF_REAL(enu.y)/100;
  enu.z = POS_BFP_OF_REAL(enu.z)/100;
  VECT3_ASSIGN(waypoints[wp_id], enu.x, enu.y, enu.z);
  DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &enu.x, &enu.y, &enu.z);
  printf("move wp id=%d x=%d y=%d z=%d\n", wp_id, enu.x, enu.y, enu.z);
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
  IvySendMsg("%d BOOZ_SIM_POS_LLH %f %f %f %f %f %f %f %f %f",
             AC_ID,
             (fdm.lla_pos_pprz.lat),
             (fdm.lla_pos_geod.lat),
             (fdm.lla_pos_geoc.lat),
             (fdm.lla_pos_pprz.lon),
             (fdm.lla_pos_geod.lon),
             (fdm.lla_pos_pprz.alt),
             (fdm.lla_pos_geod.alt),
             (fdm.agl),
             (fdm.hmsl));
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
