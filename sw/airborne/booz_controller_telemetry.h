#ifndef BOOZ_CONTROLLER_TELEMETRY_H
#define BOOZ_CONTROLLER_TELEMETRY_H

#include "std.h"
#include "messages.h"
#include "periodic.h"
#include "uart.h"

#include "radio_control.h"
#include "actuators.h"
#include "booz_link_mcu.h"
#include "booz_estimator.h"
#include "booz_autopilot.h"
#include "booz_control.h"
#include "booz_nav.h"

#include "actuators_buss_twi_blmc_hw.h"

#include "settings.h"

#include "downlink.h"

#define PERIODIC_SEND_BOOZ_STATUS()					\
  DOWNLINK_SEND_BOOZ_STATUS(&booz_link_mcu_nb_err,			\
			    &booz_link_mcu_status,			\
			    &rc_status, \
			    &booz_autopilot_mode, \
			    &booz_autopilot_mode, \
			    &cpu_time_sec, \
			    &buss_twi_blmc_nb_err)


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
      DOWNLINK_SEND_BOOZ_ATT_LOOP(&booz_estimator_phi, &booz_control_attitude_phi_sp, \
				  &booz_estimator_theta, &booz_control_attitude_theta_sp, \
				  &booz_estimator_psi, &booz_control_attitude_psi_sp, \
				  &booz_control_power_sp);		\
      break;								\
    }									\
  }

#define PERIODIC_SEND_BOOZ_UF_RATES()               \
  DOWNLINK_SEND_BOOZ_UF_RATES(&booz_estimator_uf_p, \
			      &booz_estimator_uf_q, \
			      &booz_estimator_uf_r); 

#ifndef DISABLE_NAV
#define PERIODIC_SEND_BOOZ_VERT_LOOP() {				\
    DOWNLINK_SEND_BOOZ_VERT_LOOP(&booz_nav_hover_z_sp,			\
				 &booz_estimator_w,			\
				 &booz_estimator_z,			\
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
#else
#define PERIODIC_SEND_BOOZ_VERT_LOOP() {}
#define PERIODIC_SEND_BOOZ_HOV_LOOP() {}
#endif


#define PERIODIC_SEND_BOOZ_CMDS() DOWNLINK_SEND_BOOZ_CMDS(&buss_twi_blmc_motor_power[SERVO_MOTOR_FRONT],\
							  &buss_twi_blmc_motor_power[SERVO_MOTOR_BACK],	\
							  &buss_twi_blmc_motor_power[SERVO_MOTOR_LEFT],	\
							  &buss_twi_blmc_motor_power[SERVO_MOTOR_RIGHT]); 

#define PERIODIC_SEND_DL_VALUE() PeriodicSendDlValue()

extern uint8_t telemetry_mode_Controller;

static inline void booz_controller_telemetry_periodic_task(void) {
  PeriodicSendController()
}


#endif /* BOOZ_CONTROLLER_TELEMETRY_H */
