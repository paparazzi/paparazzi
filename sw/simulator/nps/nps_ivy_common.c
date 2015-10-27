#include "nps_ivy.h"

#include <stdlib.h>
#include <stdio.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "generated/airframe.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "nps_main.h"
#include "nps_autopilot.h"
#include "nps_fdm.h"
#include "nps_sensors.h"
#include "nps_atmosphere.h"
#include "subsystems/ins.h"
#include "subsystems/navigation/common_flight_plan.h"

#if USE_GPS
#include "subsystems/gps.h"
#endif

#ifdef RADIO_CONTROL_TYPE_DATALINK
#include "subsystems/radio_control.h"
#endif

#include NPS_SENSORS_PARAMS

/* Gaia Ivy functions */
static void on_WORLD_ENV(IvyClientPtr app __attribute__((unused)),
                         void *user_data __attribute__((unused)),
                         int argc __attribute__((unused)), char *argv[]);

/* Datalink Ivy functions */
static void on_DL_SETTING(IvyClientPtr app __attribute__((unused)),
                          void *user_data __attribute__((unused)),
                          int argc __attribute__((unused)), char *argv[]);

static void on_DL_GET_SETTING(IvyClientPtr app __attribute__((unused)),
                              void *user_data __attribute__((unused)),
                              int argc __attribute__((unused)), char *argv[]);

static void on_DL_PING(IvyClientPtr app __attribute__((unused)),
                       void *user_data __attribute__((unused)),
                       int argc __attribute__((unused)), char *argv[]);

static void on_DL_BLOCK(IvyClientPtr app __attribute__((unused)),
                        void *user_data __attribute__((unused)),
                        int argc __attribute__((unused)), char *argv[]);

#ifdef RADIO_CONTROL_TYPE_DATALINK
static void on_DL_RC_3CH(IvyClientPtr app __attribute__((unused)),
                         void *user_data __attribute__((unused)),
                         int argc __attribute__((unused)), char *argv[]);

static void on_DL_RC_4CH(IvyClientPtr app __attribute__((unused)),
                         void *user_data __attribute__((unused)),
                         int argc __attribute__((unused)), char *argv[]);
#endif

void nps_ivy_common_init(char *ivy_bus)
{
  const char *agent_name = AIRFRAME_NAME"_NPS";
  const char *ready_msg = AIRFRAME_NAME"_NPS Ready";
  IvyInit(agent_name, ready_msg, NULL, NULL, NULL, NULL);

  IvyBindMsg(on_WORLD_ENV, NULL, "^(\\S*) WORLD_ENV (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");

  IvyBindMsg(on_DL_PING, NULL, "^(\\S*) DL_PING");
  IvyBindMsg(on_DL_SETTING, NULL, "^(\\S*) DL_SETTING (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_GET_SETTING, NULL, "^(\\S*) GET_DL_SETTING (\\S*) (\\S*)");
  IvyBindMsg(on_DL_BLOCK, NULL,   "^(\\S*) BLOCK (\\S*) (\\S*)");

#ifdef RADIO_CONTROL_TYPE_DATALINK
  IvyBindMsg(on_DL_RC_3CH, NULL, "^(\\S*) RC_3CH (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_DL_RC_4CH, NULL, "^(\\S*) RC_4CH (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
#endif

#ifdef __APPLE__
  const char *default_ivy_bus = "224.255.255.255";
#else
  const char *default_ivy_bus = "127.255.255.255";
#endif
  if (ivy_bus == NULL) {
    IvyStart(default_ivy_bus);
  } else {
    IvyStart(ivy_bus);
  }
}

/*
 * Parse WORLD_ENV message from gaia.
 *
 */
static void on_WORLD_ENV(IvyClientPtr app __attribute__((unused)),
                         void *user_data __attribute__((unused)),
                         int argc __attribute__((unused)), char *argv[])
{
  // wind speed in m/s
  struct FloatVect3 wind;
  wind.x = atof(argv[1]); //east
  wind.y = atof(argv[2]); //north
  wind.z = atof(argv[3]); //up

  /* set wind speed in NED */
  nps_atmosphere_set_wind_ned(wind.y, wind.x, -wind.z);

  /* not used so far */
  //float ir_contrast = atof(argv[4]);

  /* set new time factor */
  nps_set_time_factor(atof(argv[5]));

#if USE_GPS
  // directly set gps fix in subsystems/gps/gps_sim_nps.h
  gps_has_fix = atoi(argv[6]); // gps_availability
#endif
}


//TODO use datalink parsing from actual fixedwing or rotorcraft firmware,
// instead of doing it here explicitly


#include "generated/settings.h"
#include "dl_protocol.h"
#include "subsystems/datalink/downlink.h"
static void on_DL_SETTING(IvyClientPtr app __attribute__((unused)),
                          void *user_data __attribute__((unused)),
                          int argc __attribute__((unused)), char *argv[])
{
  if (atoi(argv[1]) != AC_ID) {
    return;
  }

  /* HACK:
   * we actually don't want to allow changing settings if datalink is disabled,
   * but since we currently change this variable via settings we have to allow it
   */
  //if (!autopilot.datalink_enabled)
  //  return;

  uint8_t index = atoi(argv[2]);
  float value = atof(argv[3]);
  DlSetting(index, value);
  DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &index, &value);
  printf("setting %d %f\n", index, value);
}

static void on_DL_GET_SETTING(IvyClientPtr app __attribute__((unused)),
                              void *user_data __attribute__((unused)),
                              int argc __attribute__((unused)), char *argv[])
{
  if (atoi(argv[1]) != AC_ID) {
    return;
  }
  if (!autopilot.datalink_enabled) {
    return;
  }

  uint8_t index = atoi(argv[2]);
  float value = settings_get_value(index);
  DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &index, &value);
  printf("get setting %d %f\n", index, value);
}

static void on_DL_PING(IvyClientPtr app __attribute__((unused)),
                       void *user_data __attribute__((unused)),
                       int argc __attribute__((unused)), char *argv[] __attribute__((unused)))
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
}

static void on_DL_BLOCK(IvyClientPtr app __attribute__((unused)),
                        void *user_data __attribute__((unused)),
                        int argc __attribute__((unused)), char *argv[])
{
  if (atoi(argv[2]) != AC_ID) {
    return;
  }
  if (!autopilot.datalink_enabled) {
    return;
  }

  int block = atoi(argv[1]);
  nav_goto_block(block);
  printf("goto block %d\n", block);
}

#ifdef RADIO_CONTROL_TYPE_DATALINK
static void on_DL_RC_3CH(IvyClientPtr app __attribute__((unused)),
                         void *user_data __attribute__((unused)),
                         int argc __attribute__((unused)), char *argv[])
{
  if (!autopilot.datalink_enabled) {
    return;
  }

  uint8_t throttle_mode = atoi(argv[2]);
  int8_t roll = atoi(argv[3]);
  int8_t pitch = atoi(argv[4]);
  parse_rc_3ch_datalink(throttle_mode, roll, pitch);
  //printf("rc_3ch: throttle_mode %d, roll %d, pitch %d\n", throttle_mode, roll, pitch);
}

static void on_DL_RC_4CH(IvyClientPtr app __attribute__((unused)),
                         void *user_data __attribute__((unused)),
                         int argc __attribute__((unused)), char *argv[])
{
  if (atoi(argv[1]) != AC_ID) {
    return;
  }
  if (!autopilot.datalink_enabled) {
    return;
  }

  uint8_t mode = atoi(argv[2]);
  uint8_t throttle = atoi(argv[3]);
  int8_t roll = atoi(argv[4]);
  int8_t pitch = atoi(argv[5]);
  int8_t yaw = atoi(argv[6]);
  parse_rc_4ch_datalink(mode, throttle, roll, pitch, yaw);
  //printf("rc_4ch: mode %d, throttle %d, roll %d, pitch %d, yaw %d\n", mode, throttle, roll, pitch, yaw);
}
#endif


void nps_ivy_display(void)
{
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
             DegOfRad(RATE_FLOAT_OF_BFP(sensors.gyro.bias_random_walk_value.x) + sensors.gyro.bias_initial.x),
             DegOfRad(RATE_FLOAT_OF_BFP(sensors.gyro.bias_random_walk_value.y) + sensors.gyro.bias_initial.y),
             DegOfRad(RATE_FLOAT_OF_BFP(sensors.gyro.bias_random_walk_value.z) + sensors.gyro.bias_initial.z));

  /* transform magnetic field to body frame */
  struct DoubleVect3 h_body;
  double_quat_vmult(&h_body, &fdm.ltp_to_body_quat, &fdm.ltp_h);

  IvySendMsg("%d NPS_SENSORS_SCALED %f %f %f %f %f %f",
             AC_ID,
             ((sensors.accel.value.x - sensors.accel.neutral.x) / NPS_ACCEL_SENSITIVITY_XX),
             ((sensors.accel.value.y - sensors.accel.neutral.y) / NPS_ACCEL_SENSITIVITY_YY),
             ((sensors.accel.value.z - sensors.accel.neutral.z) / NPS_ACCEL_SENSITIVITY_ZZ),
             h_body.x,
             h_body.y,
             h_body.z);

  IvySendMsg("%d NPS_WIND %f %f %f",
             AC_ID,
             fdm.wind.x,
             fdm.wind.y,
             fdm.wind.z);
}
