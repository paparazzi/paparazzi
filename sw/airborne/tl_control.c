#include "tl_control.h"

#include "tl_estimator.h"
#include "radio_control.h"


bool_t kill_throttle; /* keep this old name to get it in the GCS */

// output
float tl_control_p_sp;
float tl_control_q_sp;
float tl_control_r_sp;
float tl_control_power_sp;

float tl_control_rate_r_pgain;
float tl_control_rate_r_dgain;
float tl_control_rate_r_igain;
float tl_control_rate_last_err_r;
float tl_control_rate_sum_err_r;

float tl_control_agl_sp;



pprz_t tl_control_commands[COMMANDS_NB];

#define TL_CONTROL_RATE_R_PGAIN -120.
#define TL_CONTROL_RATE_R_DGAIN    5.
#define TL_CONTROL_RATE_R_IGAIN    0.2

/* setpoints for max stick throw in degres per second */
#define TL_CONTROL_RATE_R_MAX_SP  100.


float tl_control_attitude_phi_sp;
float tl_control_attitude_theta_sp;
float tl_control_attitude_psi_sp;

float tl_control_attitude_psi_pgain;
float tl_control_attitude_psi_dgain;
float tl_control_attitude_psi_igain;
float tl_control_attitude_psi_sum_err;


#define TL_CONTROL_ATTITUDE_PSI_PGAIN  -500.
#define TL_CONTROL_ATTITUDE_PSI_DGAIN  -150.
#define TL_CONTROL_ATTITUDE_PSI_IGAIN    -0.20
#define TL_CONTROL_ATTITUDE_PSI_TRIM    350

#define TL_CONTROL_AGL_PGAIN (-MAX_PPRZ/10) /* PPRZ / m */
#define TL_CONTROL_AGL_IGAIN 0.1 /* PPRZ / m */
#define TL_MAX_AGL_ERROR 1


/* setpoints for max stick throw in degres */
#define TL_CONTROL_ATTITUDE_PSI_MAX_SP (90. / 40.) /* called with RC messags */

#define TL_CONTROL_TRIM_R 300
int16_t tl_control_trim_r;

uint8_t tl_z_mode;
float tl_control_agl_pgain[TL_NB_Z_MODES];
float tl_control_agl_igain[TL_NB_Z_MODES];
float tl_control_agl_sum_err[TL_NB_Z_MODES];



void tl_control_init(void) {

  tl_control_p_sp = 0.;
  tl_control_q_sp = 0.;
  tl_control_r_sp = 0.;
  tl_control_power_sp = 0.;

  tl_control_rate_last_err_r = 0.;

  tl_control_rate_r_pgain = TL_CONTROL_RATE_R_PGAIN;
  tl_control_rate_r_dgain = TL_CONTROL_RATE_R_DGAIN;
  tl_control_rate_r_igain = TL_CONTROL_RATE_R_IGAIN;


  tl_control_attitude_phi_sp = 0.;
  tl_control_attitude_theta_sp =0.;
  tl_control_attitude_psi_sp =0.;
  tl_control_attitude_psi_pgain = TL_CONTROL_ATTITUDE_PSI_PGAIN;
  tl_control_attitude_psi_dgain = TL_CONTROL_ATTITUDE_PSI_DGAIN;
  tl_control_attitude_psi_igain = TL_CONTROL_ATTITUDE_PSI_IGAIN;

  tl_control_trim_r = TL_CONTROL_TRIM_R;

  tl_control_agl_pgain[TL_Z_MODE_RM] = TL_CONTROL_AGL_PGAIN;
  tl_control_agl_igain[TL_Z_MODE_RM] = TL_CONTROL_AGL_IGAIN;
  tl_control_agl_sum_err[TL_Z_MODE_RM] = 0;

  tl_z_mode = TL_Z_MODE_RC;
}


void tl_control_rate_read_setpoints_from_rc(void) {

  tl_control_p_sp = -rc_values[RADIO_ROLL];
  tl_control_q_sp =  rc_values[RADIO_PITCH];
  tl_control_r_sp = -rc_values[RADIO_YAW] * RadOfDeg(TL_CONTROL_RATE_R_MAX_SP)/MAX_PPRZ;
  //tl_control_r_sp = -rc_values[RADIO_YAW];
  tl_control_power_sp = rc_values[RADIO_THROTTLE];
}


void tl_control_rate_run(void) {

  const float cmd_p = tl_control_p_sp;
  const float cmd_q = tl_control_q_sp;

  if (estimator_in_flight) {

    const float rate_err_r = estimator_r - tl_control_r_sp;
    const float rate_d_err_r = rate_err_r - tl_control_rate_last_err_r;
    tl_control_rate_last_err_r = rate_err_r;
    tl_control_rate_sum_err_r += rate_err_r;

    const float cmd_r = - tl_control_rate_r_pgain * ( rate_err_r + 
						      tl_control_rate_r_dgain * rate_d_err_r + 
						      tl_control_rate_r_igain * tl_control_rate_sum_err_r);

    tl_control_commands[COMMAND_YAW]      = TRIM_PPRZ((int16_t)cmd_r + tl_control_trim_r);
    //  tl_control_commands[COMMAND_THROTTLE] = kill_throttle ? 0 : TRIM_PPRZ((int16_t) (tl_control_power_sp));
  }
  else {
    tl_control_rate_sum_err_r = 0.;
    tl_control_commands[COMMAND_YAW] = 0;
  }

  tl_control_commands[COMMAND_THROTTLE] = TRIM_PPRZ((int16_t) (tl_control_power_sp));
  tl_control_commands[COMMAND_ROLL]     = TRIM_PPRZ((int16_t)cmd_p);
  tl_control_commands[COMMAND_PITCH]    = TRIM_PPRZ((int16_t)cmd_q);

}

#define DO_STEPS 

void tl_control_attitude_read_setpoints_from_rc(void) {
  tl_control_attitude_phi_sp = -rc_values[RADIO_ROLL];
  tl_control_attitude_theta_sp =  rc_values[RADIO_PITCH];

  switch (tl_z_mode) {
  case TL_Z_MODE_RC:
    tl_control_power_sp = rc_values[RADIO_THROTTLE];
    break;
  case TL_Z_MODE_RM:
    tl_control_agl_sp = 2. * rc_values[RADIO_THROTTLE]/MAX_PPRZ;
    break;
  }


#ifndef DO_STEPS
  if (!estimator_in_flight)
    tl_control_attitude_psi_sp = estimator_psi;
  else {
    tl_control_attitude_psi_sp += -rc_values[RADIO_YAW] * RadOfDeg(TL_CONTROL_ATTITUDE_PSI_MAX_SP)/MAX_PPRZ;
    NormRadAngle(tl_control_attitude_psi_sp);
  }
#else
  switch (cpu_time_sec % 4) {
  case 0:
    tl_control_attitude_psi_sp = RadOfDeg(-22.5); break;
  case 1:
    tl_control_attitude_psi_sp = RadOfDeg(0.); break;
  case 2:
    tl_control_attitude_psi_sp = RadOfDeg(22.5); break;
  case 3:
    tl_control_attitude_psi_sp = RadOfDeg(0.); break;
  }
#if 0
  if (cpu_time_sec % 8 < 4)
    tl_control_attitude_psi_sp = RadOfDeg(-45.);
  else
    tl_control_attitude_psi_sp = RadOfDeg(45.);
#endif
#endif

}

void tl_control_attitude_run(void) {

  const float cmd_p = tl_control_attitude_phi_sp;
  const float cmd_q = tl_control_attitude_theta_sp;

  if (estimator_in_flight) {
    float err_psi = estimator_psi - tl_control_attitude_psi_sp;
    NormRadAngle(err_psi);
    tl_control_attitude_psi_sum_err += err_psi;

    const float cmd_r = - (tl_control_attitude_psi_pgain *  err_psi + 
			   tl_control_attitude_psi_dgain * estimator_r +
			   tl_control_attitude_psi_igain * tl_control_attitude_psi_sum_err );

    tl_control_commands[COMMAND_YAW]      = TRIM_PPRZ((int16_t)cmd_r + TL_CONTROL_ATTITUDE_PSI_TRIM);
  }
  else {
    tl_control_attitude_psi_sum_err = 0.;
    tl_control_commands[COMMAND_YAW] = 0;
  }    
  tl_control_commands[COMMAND_ROLL]     = TRIM_PPRZ((int16_t)cmd_p);
  tl_control_commands[COMMAND_PITCH]    = TRIM_PPRZ((int16_t)cmd_q);
  tl_control_commands[COMMAND_THROTTLE] = TRIM_PPRZ((int16_t) (tl_control_power_sp));
}

void tl_control_nav_read_setpoints_from_rc(void) {
  tl_control_power_sp = rc_values[RADIO_THROTTLE];
  if (!estimator_in_flight)
    tl_control_attitude_psi_sp = estimator_psi;
  else {
    tl_control_attitude_psi_sp += -rc_values[RADIO_YAW] * RadOfDeg(TL_CONTROL_ATTITUDE_PSI_MAX_SP)/MAX_PPRZ;
    NormRadAngle(tl_control_attitude_psi_sp);
  }
}

void tl_control_agl_run(void) {
  switch(tl_z_mode) {
  case TL_Z_MODE_RC:
    /* Do nothing */
    break;
  case TL_Z_MODE_RM: {
    float agl_err = tl_control_agl_sp - tl_estimator_agl;
    Bound(agl_err, -TL_MAX_AGL_ERROR, TL_MAX_AGL_ERROR);
    
    if (estimator_in_flight)
      tl_control_agl_sum_err[tl_z_mode] += agl_err;
    tl_control_power_sp = tl_estimator_cruise_power +
      tl_control_agl_pgain[tl_z_mode] * (agl_err + tl_control_agl_igain[tl_z_mode] * tl_control_agl_sum_err[tl_z_mode]);
  }
  }
}
