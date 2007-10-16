#include "booz_control.h"

#include "booz_estimator.h"
#include "commands.h"
#include "radio_control.h"

float booz_control_p_sp;
float booz_control_q_sp;
float booz_control_r_sp;

float booz_control_rate_pq_pgain;
float booz_control_rate_pq_dgain;
float booz_control_rate_r_pgain;
float booz_control_rate_r_dgain;
float booz_control_rate_last_err_p;
float booz_control_rate_last_err_q;
float booz_control_rate_last_err_r;

#define BOOZ_CONTROL_RATE_PQ_PGAIN -1000.
#define BOOZ_CONTROL_RATE_PQ_DGAIN 0.

#define BOOZ_CONTROL_RATE_R_PGAIN -100.
#define BOOZ_CONTROL_RATE_R_DGAIN 0.

/* setpoints for max stick throw in degres per second */
#define BOOZ_CONTROL_RATE_PQ_MAX_SP 200.
#define BOOZ_CONTROL_RATE_R_MAX_SP  100.

void booz_control_init(void) {

  booz_control_p_sp = 0.;
  booz_control_q_sp = 0.;
  booz_control_r_sp = 0.;
  booz_control_rate_last_err_p = 0.;
  booz_control_rate_last_err_q = 0.;
  booz_control_rate_last_err_r = 0.;

  booz_control_rate_pq_pgain = BOOZ_CONTROL_RATE_PQ_PGAIN;
  booz_control_rate_pq_dgain = BOOZ_CONTROL_RATE_PQ_DGAIN;
  booz_control_rate_r_pgain = BOOZ_CONTROL_RATE_R_PGAIN;
  booz_control_rate_r_dgain = BOOZ_CONTROL_RATE_R_DGAIN;

}


void booz_control_rate_compute_setpoints(void) {

  booz_control_p_sp = -rc_values[RADIO_ROLL]  * DegOfRad(BOOZ_CONTROL_RATE_PQ_MAX_SP)/MAX_PPRZ;
  booz_control_q_sp =  rc_values[RADIO_PITCH] * DegOfRad(BOOZ_CONTROL_RATE_PQ_MAX_SP)/MAX_PPRZ;
  booz_control_r_sp = -rc_values[RADIO_YAW]   * DegOfRad(BOOZ_CONTROL_RATE_R_MAX_SP)/MAX_PPRZ;

}


void booz_control_rate_run(void) {

  const float rate_err_p = booz_estimator_p - booz_control_p_sp;
  const float rate_d_err_p = rate_err_p - booz_control_rate_last_err_p;
  booz_control_rate_last_err_p = rate_err_p;
  const float cmd_p = booz_control_rate_pq_pgain * ( rate_err_p + booz_control_rate_pq_dgain * rate_d_err_p );

  const float rate_err_q = booz_estimator_q - booz_control_q_sp;
  const float rate_d_err_q = rate_err_q - booz_control_rate_last_err_q;
  booz_control_rate_last_err_q = rate_err_q;
  const float cmd_q = booz_control_rate_pq_pgain * ( rate_err_q + booz_control_rate_pq_dgain * rate_d_err_q );

  const float rate_err_r = booz_estimator_r - booz_control_r_sp;
  const float rate_d_err_r = rate_err_r - booz_control_rate_last_err_r;
  booz_control_rate_last_err_r = rate_err_r;
  const float cmd_r = booz_control_rate_r_pgain * ( rate_err_r + booz_control_rate_r_dgain * rate_d_err_r );

  commands[COMMAND_P] = TRIM_PPRZ((int16_t)cmd_p);
  commands[COMMAND_Q] = TRIM_PPRZ((int16_t)cmd_q);
  commands[COMMAND_R] = TRIM_PPRZ((int16_t)cmd_r);


}

void booz_control_attitude_run(void) {

}
