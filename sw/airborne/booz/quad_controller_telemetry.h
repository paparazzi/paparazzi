#ifndef BOOZ_CONTROLLER_TELEMETRY_H
#define BOOZ_CONTROLLER_TELEMETRY_H

#include "std.h"
#include "messages.h"
#include "periodic.h"
#include "uart.h"

#include "booz_energy.h"
#include "radio_control.h"
#include "actuators.h"
#include "booz_estimator.h"
#include "booz_autopilot.h"
#include "booz_control.h"
#include "booz_nav.h"
#include "quad_gps.h"

#include "settings.h"

#include "downlink.h"

#define PERIODIC_SEND_ALIVE() DOWNLINK_SEND_ALIVE(16, MD5SUM)

#define PERIODIC_SEND_ATTITUDE() { \
  int16_t phi = DegOfRad(booz_estimator_phi); \
  int16_t psi = DegOfRad(booz_estimator_psi); \
  int16_t theta = DegOfRad(booz_estimator_theta); \
  DOWNLINK_SEND_ATTITUDE(&phi, &psi, &theta); \
}

#define PERIODIC_SEND_ESTIMATOR() DOWNLINK_SEND_ESTIMATOR(&booz_estimator_z, &booz_estimator_vz)

#define PERIODIC_SEND_BAT() { \
  int16_t t = 0; \
  uint16_t time = 0; \
  uint8_t kill = 0; if (booz_autopilot_mode == BOOZ_AP_MODE_KILL) kill = 1; \
  DOWNLINK_SEND_BAT(&t, &booz_energy_bat, &booz_estimator_flight_time, &kill, &time, &time, &booz_energy_current); \
}

#define PERIODIC_SEND_PPRZ_MODE() { \
  uint8_t pprz_mode; \
  switch (booz_autopilot_mode) { \
    case BOOZ_AP_MODE_RATE: \
      pprz_mode = 0; /*MANUAL*/ break; \
    case BOOZ_AP_MODE_ATTITUDE: \
    case BOOZ_AP_MODE_HEADING_HOLD: \
      pprz_mode = 1; /*AUTO1*/ break; \
    case BOOZ_AP_MODE_NAV: \
      pprz_mode = 2; /*AUTO2*/ break; \
    case BOOZ_AP_MODE_FAILSAFE: \
    case BOOZ_AP_MODE_KILL: \
      pprz_mode = 5; /*FAIL*/ break; \
  } \
  uint8_t gaz = 0; \
  uint8_t lat = 0; \
  uint8_t hor = 0; \
  uint8_t calib = 0; \
  uint8_t rc; \
  switch (rc_status) { \
    case RC_OK: \
      rc = 1; /*OK*/ break; \
    case RC_LOST: \
      rc = 2; /*LOST*/ break; \
    case RC_REALLY_LOST: \
      rc = 0; /*NONE*/ break; \
  } \
  DOWNLINK_SEND_PPRZ_MODE(&pprz_mode, &gaz, &lat, &hor, &calib, &rc); \
}

#define PERIODIC_SEND_BOOZ_STATUS() \
  DOWNLINK_SEND_QUAD_STATUS(			\
			    &rc_status,					\
			    &booz_autopilot_mode,			\
			    &booz_energy_bat,				\
			    &cpu_time_sec)

#define PERIODIC_SEND_RC() DOWNLINK_SEND_RC(PPM_NB_PULSES, rc_values)

#define PERIODIC_SEND_BOOZ_FD()						\
  DOWNLINK_SEND_BOOZ_FD(&booz_estimator_p,				\
			&booz_estimator_q,				\
			&booz_estimator_r,				\
			&booz_estimator_phi,				\
			&booz_estimator_theta,				\
			&booz_estimator_psi); 

#define PERIODIC_SEND_ACTUATORS()			\
  DOWNLINK_SEND_ACTUATORS(SERVOS_NB, actuators);

#define PERIODIC_SEND_BOOZ_CONTROL() {					\
    switch (booz_autopilot_mode) {					\
    case BOOZ_AP_MODE_RATE:						\
      DOWNLINK_SEND_BOOZ_RATE_LOOP(&booz_estimator_uf_p, &booz_control_p_sp, \
				   &booz_estimator_uf_q, &booz_control_q_sp, \
				   &booz_estimator_uf_r, &booz_control_r_sp, \
				   &booz_control_power_sp);		\
      break;								\
    case BOOZ_AP_MODE_ATTITUDE:						\
    case BOOZ_AP_MODE_HEADING_HOLD:					\
    case BOOZ_AP_MODE_NAV:						\
      DOWNLINK_SEND_BOOZ_ATT_LOOP(&booz_estimator_phi, &booz_control_attitude_phi_sp, \
				  &booz_estimator_theta, &booz_control_attitude_theta_sp, \
				  &booz_estimator_psi, &booz_control_attitude_psi_sp, \
				  &booz_control_power_sp);		\
      break;								\
    }									\
  }

#define PERIODIC_SEND_BOOZ_VERT_LOOP() {				\
    float sp = -booz_nav_hover_z_sp; \
    float z = -booz_estimator_z; \
    float vz = -booz_estimator_w; \
    DOWNLINK_SEND_BOOZ_VERT_LOOP(&sp,&vz,&z, \
				 &booz_nav_hover_power_command);	\
  }

#define PERIODIC_SEND_BOOZ_HOV_LOOP() {					\
    DOWNLINK_SEND_BOOZ_HOV_LOOP(&booz_nav_hover_x_sp,			\
				&booz_nav_hover_y_sp,			\
				&booz_estimator_u,			\
				&booz_estimator_x,			\
				&booz_estimator_v,			\
				&booz_estimator_y,			\
				&booz_nav_hover_phi_command,		\
				&booz_nav_hover_theta_command);		\
  }

#define PERIODIC_SEND_BOOZ_CMDS() DOWNLINK_SEND_BOOZ_CMDS(&actuators[SERVO_MOTOR_FRONT],&actuators[SERVO_MOTOR_BACK],&actuators[SERVO_MOTOR_RIGHT],&actuators[SERVO_MOTOR_LEFT])

#define PERIODIC_SEND_DL_VALUE() PeriodicSendDlValue()

#define PERIODIC_SEND_GPS() gps_downlink();

extern uint8_t telemetry_mode_Controller;

static inline void booz_controller_telemetry_periodic_task(void) {
  PeriodicSendController()
}


#endif /* BOOZ_CONTROLLER_TELEMETRY_H */
