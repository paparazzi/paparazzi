#ifndef TL_CONTROL_H
#define TL_CONTROL_H

#include "std.h"
#include "paparazzi.h"

extern void tl_control_init(void);

extern void tl_control_rate_read_setpoints_from_rc(void);
extern void tl_control_rate_run(void);

extern void tl_control_attitude_read_setpoints_from_rc(void);
extern void tl_control_attitude_run(void);
extern void tl_control_agl_run(void);
extern void tl_control_nav_read_setpoints_from_rc(void);

extern bool_t kill_throttle;

extern float tl_control_p_sp;
extern float tl_control_q_sp;
extern float tl_control_r_sp;
extern float tl_control_power_sp;

extern float tl_control_rate_pq_pgain;
extern float tl_control_rate_pq_dgain;
extern float tl_control_rate_r_pgain;
extern float tl_control_rate_r_dgain;
extern float tl_control_rate_r_igain;
extern float tl_control_rate_sum_err_r;


extern float tl_control_attitude_phi_sp;
extern float tl_control_attitude_theta_sp;
extern float tl_control_attitude_psi_sp;

extern float tl_control_attitude_phi_theta_pgain;
extern float tl_control_attitude_phi_theta_dgain;
extern float tl_control_attitude_psi_pgain;
extern float tl_control_attitude_psi_dgain;
extern float tl_control_attitude_psi_igain;
extern float tl_control_attitude_psi_sum_err;

extern int16_t tl_control_trim_r;

#define TlControlAttitudeSetSetPoints(_phi_sp, _theta_sp, _psi_sp, _power_sp) { \
    tl_control_attitude_phi_sp = _phi_sp;				          \
    tl_control_attitude_theta_sp = _theta_sp;				          \
    tl_control_attitude_psi_sp = _psi_sp;						  \
    tl_control_power_sp = _power_sp;					          \
  }

#include "airframe.h"
extern pprz_t tl_control_commands[COMMANDS_NB];

#endif /* TL_CONTROL_H */
