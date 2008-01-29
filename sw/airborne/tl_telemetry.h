#ifndef TL_TELEMETRY_H

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
#include "tl_imu.h"
#include "tl_nav.h"
#include "tl_control.h"

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


#define PERIODIC_SEND_IMU_GYRO() DOWNLINK_SEND_IMU_GYRO(&tl_imu_r, &tl_imu_r, &tl_imu_r)
#define PERIODIC_SEND_IMU_MAG() DOWNLINK_SEND_IMU_MAG(&tl_imu_hx, &tl_imu_hy, &tl_imu_hz)

#define PERIODIC_SEND_TL_ESTIMATOR() DOWNLINK_SEND_TL_ESTIMATOR(&estimator_r, &estimator_psi, &estimator_z_baro)

#define PERIODIC_SEND_RATE_LOOP() DOWNLINK_SEND_BOOZ_RATE_LOOP(&estimator_r, &tl_control_r_sp, &estimator_r, &tl_control_r_sp, &estimator_r, &tl_control_r_sp)

#if 0
#define PERIODIC_SEND_TL_ESTIMATOR() {				\
    float d1 = tl_baro_d[0];						\
    float d2 = tl_baro_d[1];						\
    DOWNLINK_SEND_TL_ESTIMATOR(&estimator_z_baro, &d1, &d2);		\
  }
#endif

extern uint8_t telemetry_mode_Ap;

static inline void tl_telemetry_periodic_task(void) {
  PeriodicSendAp()
}

#define TL_TELEMETRY_H
#endif //TL_TELEMETRY_H
