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

  booz2_main_init();

}

#include <stdio.h>
#include "booz2_gps.h"

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
    //   double hover = 0.23;
    //  double hover = 0.;
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
#if 0
    int32_t ut_front = supervision_commands[SERVO_FRONT] - TRIM_FRONT;
    int32_t ut_back  = Actuator(SERVO_BACK)  - TRIM_BACK;
    int32_t ut_right = Actuator(SERVO_RIGHT) - TRIM_RIGHT;
    int32_t ut_left  = Actuator(SERVO_LEFT)  - TRIM_LEFT;
    autopilot.commands[SERVO_FRONT] = (double)ut_front / SUPERVISION_MAX_MOTOR;
    autopilot.commands[SERVO_BACK]  = (double)ut_back  / SUPERVISION_MAX_MOTOR;
    autopilot.commands[SERVO_RIGHT] = (double)ut_right / SUPERVISION_MAX_MOTOR;
    autopilot.commands[SERVO_LEFT]  = (double)ut_left  / SUPERVISION_MAX_MOTOR;
#endif
  }
  //  printf("%f %f %f %f\n", autopilot.commands[SERVO_FRONT], autopilot.commands[SERVO_BACK],
  //                          autopilot.commands[SERVO_RIGHT], autopilot.commands[SERVO_LEFT]);
}

#include "nps_fdm.h"
#include "math/pprz_algebra_int.h"
#include "booz_ahrs.h"
void sim_overwrite_ahrs(void) {

  //  printf("%f\n", fdm.ltpprz_to_body_eulers.phi);

  //  printf("filter theta %d  sim %f\n", booz_ahrs.ltp_to_body_euler.theta, fdm.ltp_to_body_eulers.theta);
  //  printf("filter qy %d  sim %f\n", booz_ahrs.ltp_to_body_quat.qy, fdm.ltp_to_body_quat.qy);

  booz_ahrs.ltp_to_body_euler.phi   = ANGLE_BFP_OF_REAL(fdm.ltp_to_body_eulers.phi);
  booz_ahrs.ltp_to_body_euler.theta = ANGLE_BFP_OF_REAL(fdm.ltp_to_body_eulers.theta);
  booz_ahrs.ltp_to_body_euler.psi   = ANGLE_BFP_OF_REAL(fdm.ltp_to_body_eulers.psi);

  booz_ahrs.ltp_to_body_quat.qi = QUAT1_BFP_OF_REAL(fdm.ltp_to_body_quat.qi);
  booz_ahrs.ltp_to_body_quat.qx = QUAT1_BFP_OF_REAL(fdm.ltp_to_body_quat.qx);
  booz_ahrs.ltp_to_body_quat.qy = QUAT1_BFP_OF_REAL(fdm.ltp_to_body_quat.qy);
  booz_ahrs.ltp_to_body_quat.qz = QUAT1_BFP_OF_REAL(fdm.ltp_to_body_quat.qz);

  booz_ahrs.body_rate.p = RATE_BFP_OF_REAL(fdm.body_ecef_rotvel.p);
  booz_ahrs.body_rate.q = RATE_BFP_OF_REAL(fdm.body_ecef_rotvel.q);
  booz_ahrs.body_rate.r = RATE_BFP_OF_REAL(fdm.body_ecef_rotvel.r);

  INT32_RMAT_OF_QUAT(booz_ahrs.ltp_to_body_rmat, booz_ahrs.ltp_to_body_quat);

}

