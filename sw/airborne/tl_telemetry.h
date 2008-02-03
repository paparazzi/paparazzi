#ifndef TL_TELEMETRY_H
#define TL_TELEMETRY_H

#include "std.h"
#include "messages.h"
#include "periodic.h"
#include "uart.h"
#include "downlink.h"
#include "radio_control.h"
#include "commands.h"
#include "actuators.h"
#include "tl_bat.h"
#include "tl_estimator.h"
#include "tl_vfilter.h"
#include "tl_imu.h"
#include "tl_nav.h"
#include "tl_control.h"
#include "tl_autopilot.h"

#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE

#define PERIODIC_SEND_ALIVE() DOWNLINK_SEND_ALIVE()
#define PERIODIC_SEND_RC() DOWNLINK_SEND_RC(PPM_NB_PULSES, rc_values)
#define PERIODIC_SEND_PPM() DOWNLINK_SEND_PPM(PPM_NB_PULSES, ppm_pulses)
#define PERIODIC_SEND_COMMANDS() DOWNLINK_SEND_COMMANDS(COMMANDS_NB, commands)
#define PERIODIC_SEND_ACTUATORS() DOWNLINK_SEND_ACTUATORS(SERVOS_NB, actuators)
#define PERIODIC_SEND_BAT() { int16_t dummy16=42; DOWNLINK_SEND_BAT(&commands[COMMAND_THROTTLE], &tl_bat_decivolt, &estimator_flight_time, &kill_throttle, &block_time, &stage_time, &dummy16); }
#define PERIODIC_SEND_ESTIMATOR() DOWNLINK_SEND_ESTIMATOR(&estimator_z, &estimator_climb)
#define PERIODIC_SEND_NAVIGATION_REF()  DOWNLINK_SEND_NAVIGATION_REF(&nav_utm_east0, &nav_utm_north0, &nav_utm_zone0);

#define DownlinkSendWp(i) { \
  float x = nav_utm_east0 +  waypoints[i].x; \
  float y = nav_utm_north0 + waypoints[i].y; \
  DOWNLINK_SEND_WP_MOVED(&i, &x, &y, &(waypoints[i].a),&nav_utm_zone0); \
}
#define PERIODIC_SEND_WP_MOVED() { \
  static uint8_t i; \
  i++; if (i >= nb_waypoint) i = 0; \
  DownlinkSendWp(i); \
}

#define SEND_NAVIGATION() { int16_t pos_x = estimator_x; int16_t pos_y = estimator_y; DOWNLINK_SEND_NAVIGATION(&nav_block, &nav_stage, &pos_x, &pos_y, &dist2_to_wp, &dist2_to_home);}
#define PERIODIC_SEND_NAVIGATION() SEND_NAVIGATION()


#define PERIODIC_SEND_IMU_ACCEL()      DOWNLINK_SEND_TL_IMU_ACCEL(&tl_imu_accel)
#define PERIODIC_SEND_IMU_GYRO()       DOWNLINK_SEND_TL_IMU_GYRO(&tl_imu_r)
#define PERIODIC_SEND_IMU_MAG()        DOWNLINK_SEND_TL_IMU_MAG(&tl_imu_hx, &tl_imu_hy, &tl_imu_hz)
#define PERIODIC_SEND_IMU_PRESSURE()   DOWNLINK_SEND_TL_IMU_PRESSURE(&tl_imu_pressure)
#define PERIODIC_SEND_IMU_RANGEMETER() DOWNLINK_SEND_TL_IMU_RANGEMETER(&tl_imu_rm)


#define PERIODIC_SEND_TL_ESTIMATOR() { DOWNLINK_SEND_TL_ESTIMATOR(&estimator_r, &estimator_psi, &estimator_z_baro)}

#define PERIODIC_SEND_LOOP() {						\
    switch (tl_autopilot_mode) {					\
    case TL_AP_MODE_RATE :						\
      DOWNLINK_SEND_TL_RATE_LOOP_R(&estimator_r, &tl_control_r_sp, &tl_control_rate_sum_err_r); \
      break;								\
    case TL_AP_MODE_ATTITUDE : 						\
      DOWNLINK_SEND_TL_ATTITUDE_LOOP_R(&estimator_psi, &tl_control_attitude_psi_sp, &tl_control_attitude_psi_sum_err, &estimator_r); \
      break;								\
    case TL_AP_MODE_NAV : 						\
      DOWNLINK_SEND_BOOZ_HOV_LOOP(&tl_nav_goto_x_sp, &tl_nav_goto_y_sp, &tl_estimator_u, &estimator_x, &tl_estimator_v, &estimator_y, & tl_control_attitude_phi_sp, & tl_control_attitude_theta_sp); \
      break;								\
    }									\
  }

#define PERIODIC_SEND_TL_VERTICAL_LOOP() DOWNLINK_SEND_TL_VERTICAL_LOOP(&tl_estimator_agl, &tl_control_agl_sp, &tl_control_agl_sum_err, &tl_estimator_agl_dot, &tl_estimator_cruise_power)

#define PERIODIC_SEND_TL_DEBUG() DOWNLINK_SEND_TL_DEBUG(&x_unit_err_body, &y_unit_err_body)


#define PERIODIC_SEND_TL_KALM_PSI_STATE() {				\
    DOWNLINK_SEND_TL_KALM_PSI_STATE(&tl_psi_kalm_psi, &tl_psi_kalm_bias); \
  }

#define PERIODIC_SEND_TL_KALM_PSI_COV() {				\
    DOWNLINK_SEND_TL_KALM_PSI_COV(&tl_psi_kalm_P[0][0], &tl_psi_kalm_P[0][1], &tl_psi_kalm_P[1][1]); \
  }


#define PERIODIC_SEND_TL_KALM_V_STATE() {				\
    DOWNLINK_SEND_TL_KALM_V_STATE(&tl_vf_z, &tl_vf_zdot, &tl_vf_bias, &tl_vf_z_meas); \
  }

#define PERIODIC_SEND_TL_KALM_V_COV() {				\
    DOWNLINK_SEND_TL_KALM_V_COV(&tl_vf_P[0][0], &tl_vf_P[1][1], &tl_vf_P[2][2]); \
  }





extern uint8_t telemetry_mode_Ap;

static inline void tl_telemetry_periodic_task(void) {
  PeriodicSendAp()
}

#endif //TL_TELEMETRY_H
