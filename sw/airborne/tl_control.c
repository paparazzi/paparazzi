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
float tl_control_rate_last_err_r;

pprz_t tl_control_commands[COMMANDS_NB];

#define TL_CONTROL_RATE_R_PGAIN -600.
#define TL_CONTROL_RATE_R_DGAIN    5.

/* setpoints for max stick throw in degres per second */
#define TL_CONTROL_RATE_R_MAX_SP  100.


/* float tl_control_attitude_phi_sp; */
/* float tl_control_attitude_theta_sp; */
/* float tl_control_attitude_psi_sp; */
/* float tl_control_attitude_phi_theta_pgain; */
/* float tl_control_attitude_phi_theta_dgain; */
/* float tl_control_attitude_psi_pgain; */
/* float tl_control_attitude_psi_dgain; */

/* #define TL_CONTROL_ATTITUDE_PHI_THETA_PGAIN  -1250. */
/* #define TL_CONTROL_ATTITUDE_PHI_THETA_DGAIN   -700. */

/* #define TL_CONTROL_ATTITUDE_PSI_PGAIN  -1050. */
/* #define TL_CONTROL_ATTITUDE_PSI_DGAIN   -850. */

/* setpoints for max stick throw in degres */
/* #define TL_CONTROL_ATTITUDE_PHI_THETA_MAX_SP 30. */
/* #define TL_CONTROL_ATTITUDE_PSI_MAX_SP 45. */


void tl_control_init(void) {

  tl_control_p_sp = 0.;
  tl_control_q_sp = 0.;
  tl_control_r_sp = 0.;
  tl_control_power_sp = 0.;

  tl_control_rate_last_err_r = 0.;

  tl_control_rate_r_pgain = TL_CONTROL_RATE_R_PGAIN;
  tl_control_rate_r_dgain = TL_CONTROL_RATE_R_DGAIN;


/*   tl_control_attitude_phi_sp = 0.; */
/*   tl_control_attitude_theta_sp =0.; */
/*   tl_control_attitude_psi_sp =0.; */
/*   tl_control_attitude_phi_theta_pgain = TL_CONTROL_ATTITUDE_PHI_THETA_PGAIN; */
/*   tl_control_attitude_phi_theta_dgain = TL_CONTROL_ATTITUDE_PHI_THETA_DGAIN; */
/*   tl_control_attitude_psi_pgain = TL_CONTROL_ATTITUDE_PSI_PGAIN; */
/*   tl_control_attitude_psi_dgain = TL_CONTROL_ATTITUDE_PSI_DGAIN; */

}


void tl_control_rate_read_setpoints_from_rc(void) {

  tl_control_p_sp = -rc_values[RADIO_ROLL];
  tl_control_q_sp =  rc_values[RADIO_PITCH];
  //  tl_control_r_sp = -rc_values[RADIO_YAW] * RadOfDeg(TL_CONTROL_RATE_R_MAX_SP)/MAX_PPRZ;
  tl_control_r_sp = -rc_values[RADIO_YAW];
  tl_control_power_sp = rc_values[RADIO_THROTTLE];

}


void tl_control_rate_run(void) {

  const float cmd_p = tl_control_p_sp;
  const float cmd_q = tl_control_q_sp;

  //  const float rate_err_r = tl_estimator_uf_r - tl_control_r_sp;
  //  const float rate_d_err_r = rate_err_r - tl_control_rate_last_err_r;
  //  tl_control_rate_last_err_r = rate_err_r;
  //  const float cmd_r = tl_control_rate_r_pgain * ( rate_err_r + tl_control_rate_r_dgain * rate_d_err_r );
  const float cmd_r = tl_control_r_sp;

  tl_control_commands[COMMAND_ROLL]     = TRIM_PPRZ((int16_t)cmd_p);
  tl_control_commands[COMMAND_PITCH]    = TRIM_PPRZ((int16_t)cmd_q);
  tl_control_commands[COMMAND_YAW]      = TRIM_PPRZ((int16_t)cmd_r);
  tl_control_commands[COMMAND_THROTTLE] = kill_throttle ? 0 : TRIM_PPRZ((int16_t) (tl_control_power_sp));

}

void tl_control_attitude_read_setpoints_from_rc(void) {
 
}

void tl_control_attitude_run(void) {

}
