#include "nps_autopilot_booz.h"

#include "booz2_main.h"
#include "nps_sensors.h"
#include "nps_radio_control.h"
#include "booz_radio_control.h"
#include "booz_imu.h"
#include "booz2_analog_baro.h"

#include "actuators/booz_supervision.h"


struct NpsAutopilot autopilot;
bool_t nps_bypass_ahrs;


void nps_autopilot_init(enum NpsRadioControlType type_rc, int num_rc_script, char* rc_dev) {

  nps_radio_control_init(type_rc, num_rc_script, rc_dev);
  nps_bypass_ahrs = TRUE;
  //  nps_bypass_ahrs = FALSE;

  booz2_main_init();

}

#include <stdio.h>
#include "booz_gps.h"

void nps_autopilot_run_step(double time __attribute__ ((unused))) {

  if (nps_radio_control_available(time)) {
    booz_radio_control_feed();
    booz2_main_event();
  }

  if (nps_sensors_gyro_available()) {
    booz_imu_feed_gyro_accel();
    booz2_main_event();
  }

  if (nps_sensors_mag_available()) {
    booz_imu_feed_mag();
    booz2_main_event();
 }

  if (nps_sensors_baro_available()) {
    Booz2BaroISRHandler(sensors.baro.value);
    booz2_main_event();
  }

  if (nps_sensors_gps_available()) {
    booz_gps_feed_value();
    booz2_main_event();
  }

  if (nps_bypass_ahrs) {
    sim_overwrite_ahrs();
  }

  booz2_main_periodic();

  /* 25 */
  if (time < 8) {
    //    double hover = 0.25;
    double hover = 0.2493;
    //  if (time > 20) hover = 0.25;
    double yaw = 0.000000;
    double pitch = 0.000;
    double roll  = 0.0000;

    autopilot.commands[SERVO_FRONT] = hover + yaw + pitch;
    autopilot.commands[SERVO_BACK]  = hover + yaw - pitch;
    autopilot.commands[SERVO_RIGHT] = hover - yaw - roll ;
    autopilot.commands[SERVO_LEFT]  = hover - yaw + roll;
  }
  else {
    uint8_t i;
    for (i=0; i<ACTUATORS_MKK_NB; i++)
      autopilot.commands[i] = (double)supervision.commands[i] / SUPERVISION_MAX_MOTOR;
  }
  //  printf("%f %f %f %f\n", autopilot.commands[SERVO_FRONT], autopilot.commands[SERVO_BACK],
  //                          autopilot.commands[SERVO_RIGHT], autopilot.commands[SERVO_LEFT]);
}

#include "nps_fdm.h"
#include "booz_ahrs.h"
#include "math/pprz_algebra.h"
void sim_overwrite_ahrs(void) {

  EULERS_BFP_OF_REAL(booz_ahrs.ltp_to_body_euler, fdm.ltp_to_body_eulers);
  
  QUAT_BFP_OF_REAL(booz_ahrs.ltp_to_body_quat, fdm.ltp_to_body_quat);
  
  RATES_BFP_OF_REAL(booz_ahrs.body_rate, fdm.body_ecef_rotvel);
  
  INT32_RMAT_OF_QUAT(booz_ahrs.ltp_to_body_rmat, booz_ahrs.ltp_to_body_quat);
  
}

