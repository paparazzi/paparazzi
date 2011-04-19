#include "nps_autopilot_booz.h"

#include "firmwares/rotorcraft/main.h"
#include "nps_sensors.h"
#include "nps_radio_control.h"
#include "subsystems/radio_control.h"
#include "subsystems/imu.h"
#include "subsystems/sensors/baro.h"
#include "baro_board.h"
#include "subsystems/electrical.h"

#include "actuators/supervision.h"


struct NpsAutopilot autopilot;
bool_t nps_bypass_ahrs;


void nps_autopilot_init(enum NpsRadioControlType type_rc, int num_rc_script, char* rc_dev) {

  nps_radio_control_init(type_rc, num_rc_script, rc_dev);
  nps_bypass_ahrs = TRUE;
  //  nps_bypass_ahrs = FALSE;

  main_init();

#ifdef MAX_BAT_LEVEL
  electrical.vsupply = MAX_BAT_LEVEL * 10;
#else
  electrical.vsupply = 111;
#endif

}

#include <stdio.h>
#include "booz_gps.h"

void nps_autopilot_run_step(double time __attribute__ ((unused))) {

  if (nps_radio_control_available(time)) {
    radio_control_feed();
    main_event();
  }

  if (nps_sensors_gyro_available()) {
    imu_feed_gyro_accel();
    main_event();
  }

  if (nps_sensors_mag_available()) {
    imu_feed_mag();
    main_event();
 }

  if (nps_sensors_baro_available()) {
    baro_feed_value(sensors.baro.value);
    main_event();
  }

  if (nps_sensors_gps_available()) {
    booz_gps_feed_value();
    main_event();
  }

  if (nps_bypass_ahrs) {
    sim_overwrite_ahrs();
  }

  main_periodic();

  if (time < 8) { /* start with a little bit of hovering */
    int32_t init_cmd[4];
    init_cmd[COMMAND_THRUST] = 0.253*SUPERVISION_MAX_MOTOR;
    init_cmd[COMMAND_ROLL]   = 0;
    init_cmd[COMMAND_PITCH]  = 0;
    init_cmd[COMMAND_YAW]    = 0;
    supervision_run(TRUE, FALSE, init_cmd);
  }
  for (uint8_t i=0; i<ACTUATORS_MKK_NB; i++)
    autopilot.commands[i] = (double)supervision.commands[i] / SUPERVISION_MAX_MOTOR;

}

#include "nps_fdm.h"
#include "subsystems/ahrs.h"
#include "math/pprz_algebra.h"
void sim_overwrite_ahrs(void) {

  EULERS_BFP_OF_REAL(ahrs.ltp_to_body_euler, fdm.ltp_to_body_eulers);

  QUAT_BFP_OF_REAL(ahrs.ltp_to_body_quat, fdm.ltp_to_body_quat);

  RATES_BFP_OF_REAL(ahrs.body_rate, fdm.body_ecef_rotvel);

  INT32_RMAT_OF_QUAT(ahrs.ltp_to_body_rmat, ahrs.ltp_to_body_quat);

}
