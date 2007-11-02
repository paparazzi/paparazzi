#ifndef BOOZ_CONTROL_H
#define BOOZ_CONTROL_H

#include "std.h"
#include "paparazzi.h"

extern void booz_control_init(void);

extern void booz_control_rate_read_setpoints_from_rc(void);
extern void booz_control_rate_run(void);

extern void booz_control_attitude_read_setpoints_from_rc(void);
extern void booz_control_attitude_run(void);

extern float booz_control_p_sp;
extern float booz_control_q_sp;
extern float booz_control_r_sp;
extern float booz_control_power_sp;

extern float booz_control_rate_pq_pgain;
extern float booz_control_rate_pq_dgain;
extern float booz_control_rate_r_pgain;
extern float booz_control_rate_r_dgain;

extern float booz_control_attitude_phi_sp;
extern float booz_control_attitude_theta_sp;
extern float booz_control_attitude_psi_sp;

extern float booz_control_attitude_phi_theta_pgain;
extern float booz_control_attitude_phi_theta_dgain;
extern float booz_control_attitude_psi_pgain;
extern float booz_control_attitude_psi_dgain;


#define BoozControlAttitudeSetSetPoints(_phi_sp, _theta_sp, _psi_sp, _power_sp) { \
    booz_control_attitude_phi_sp = _phi_sp;				          \
    booz_control_attitude_theta_sp = _theta_sp;				          \
    booz_control_attitude_psi_sp = _psi_sp;						  \
    booz_control_power_sp = _power_sp;					          \
  }

#include "airframe.h"
extern pprz_t booz_control_commands[COMMANDS_NB];

#endif /* BOOZ_CONTROL_H */
