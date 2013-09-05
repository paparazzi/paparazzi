#include "nps_ivy.h"

#include <stdlib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "generated/airframe.h"
#include "math/pprz_algebra_double.h"
#include "nps_autopilot.h"
#include "nps_fdm.h"
#include "nps_sensors.h"
#include "subsystems/ins.h"
#include "subsystems/navigation/common_flight_plan.h"

#ifdef RADIO_CONTROL_TYPE_DATALINK
#include "subsystems/radio_control.h"
#endif

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

#ifdef RADIO_CONTROL_TYPE_DATALINK
static void on_DL_RC_3CH(IvyClientPtr app __attribute__ ((unused)),
                         void *user_data __attribute__ ((unused)),
                         int argc __attribute__ ((unused)), char *argv[]);

static void on_DL_RC_4CH(IvyClientPtr app __attribute__ ((unused)),
                         void *user_data __attribute__ ((unused)),
                         int argc __attribute__ ((unused)), char *argv[]);
#endif

void nps_ivy_common_init(char* ivy_bus) {
  const char* agent_name = AIRFRAME_NAME"_NPS";
  const char* ready_msg = AIRFRAME_NAME"_NPS Ready";
  IvyInit(agent_name, ready_msg, NULL, NULL, NULL, NULL);
  IvyBindMsg(on_DL_PING, NULL, "^(\\S*) DL_PING");
  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_GET_SETTING, NULL, "^(\\S*) GET_DL_SETTING (\\S*) (\\S*)");
  IvyBindMsg(on_DL_BLOCK, NULL,   "^(\\S*) BLOCK (\\S*) (\\S*)");

#ifdef RADIO_CONTROL_TYPE_DATALINK
  IvyBindMsg(on_DL_RC_3CH, NULL, "^(\\S*) RC_3CH (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_RC_4CH, NULL, "^(\\S*) RC_4CH (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
#endif

#ifdef __APPLE__
  const char* default_ivy_bus = "224.255.255.255";
#else
  const char* default_ivy_bus = "127.255.255.255";
#endif
  if (ivy_bus == NULL) {
    IvyStart(default_ivy_bus);
  } else {
    IvyStart(ivy_bus);
  }
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

#ifdef RADIO_CONTROL_TYPE_DATALINK
static void on_DL_RC_3CH(IvyClientPtr app __attribute__ ((unused)),
                         void *user_data __attribute__ ((unused)),
                         int argc __attribute__ ((unused)), char *argv[]){
  uint8_t throttle_mode = atoi(argv[2]);
  int8_t roll = atoi(argv[3]);
  int8_t pitch = atoi(argv[4]);
  parse_rc_3ch_datalink(throttle_mode, roll, pitch);
  //printf("rc_3ch: throttle_mode %d, roll %d, pitch %d\n", throttle_mode, roll, pitch);
}

static void on_DL_RC_4CH(IvyClientPtr app __attribute__ ((unused)),
                         void *user_data __attribute__ ((unused)),
                         int argc __attribute__ ((unused)), char *argv[]){
  uint8_t mode = atoi(argv[2]);
  uint8_t throttle = atoi(argv[3]);
  int8_t roll = atoi(argv[4]);
  int8_t pitch = atoi(argv[5]);
  int8_t yaw = atoi(argv[6]);
  parse_rc_4ch_datalink(mode, throttle, roll, pitch, yaw);
  //printf("rc_4ch: mode %d, throttle %d, roll %d, pitch %d, yaw %d\n", mode, throttle, roll, pitch, yaw);
}
#endif


void nps_ivy_display(void) {
  IvySendMsg("%d NPS_RATE_ATTITUDE %f %f %f %f %f %f",
             AC_ID,
             DegOfRad(fdm.body_ecef_rotvel.p),
             DegOfRad(fdm.body_ecef_rotvel.q),
             DegOfRad(fdm.body_ecef_rotvel.r),
             DegOfRad(fdm.ltp_to_body_eulers.phi),
             DegOfRad(fdm.ltp_to_body_eulers.theta),
             DegOfRad(fdm.ltp_to_body_eulers.psi));
  IvySendMsg("%d NPS_POS_LLH %f %f %f %f %f %f %f %f %f",
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
  IvySendMsg("%d NPS_SPEED_POS %f %f %f %f %f %f %f %f %f",
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
  IvySendMsg("%d NPS_GYRO_BIAS %f %f %f",
             AC_ID,
             DegOfRad(RATE_FLOAT_OF_BFP(sensors.gyro.bias_random_walk_value.x)+sensors.gyro.bias_initial.x),
             DegOfRad(RATE_FLOAT_OF_BFP(sensors.gyro.bias_random_walk_value.y)+sensors.gyro.bias_initial.y),
             DegOfRad(RATE_FLOAT_OF_BFP(sensors.gyro.bias_random_walk_value.z)+sensors.gyro.bias_initial.z));

  /* transform magnetic field to body frame */
  struct DoubleVect3 h_body;
  FLOAT_QUAT_VMULT(h_body, fdm.ltp_to_body_quat, fdm.ltp_h);

  IvySendMsg("%d NPS_SENSORS_SCALED %f %f %f %f %f %f",
         AC_ID,
         ((sensors.accel.value.x - sensors.accel.neutral.x)/NPS_ACCEL_SENSITIVITY_XX),
         ((sensors.accel.value.y - sensors.accel.neutral.y)/NPS_ACCEL_SENSITIVITY_YY),
         ((sensors.accel.value.z - sensors.accel.neutral.z)/NPS_ACCEL_SENSITIVITY_ZZ),
         h_body.x,
         h_body.y,
         h_body.z);
}
