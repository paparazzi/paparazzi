#ifndef BOOZ_CONTROL_H
#define BOOZ_CONTROL_H

#include "std.h"
#include "paparazzi.h"

extern void booz_control_init(void);

extern void booz_control_rate_compute_setpoints(void);
extern void booz_control_rate_run(void);

extern void booz_control_attitude_get_sp(void);
extern void booz_control_attitude_run(void);

extern float booz_control_p_sp;
extern float booz_control_q_sp;
extern float booz_control_r_sp;

extern float booz_control_rate_pq_pgain;
extern float booz_control_rate_pq_dgain;
extern float booz_control_rate_r_pgain;
extern float booz_control_rate_r_dgain;

extern pprz_t booz_control_command_p;
extern pprz_t booz_control_command_q;
extern pprz_t booz_control_command_r;

#endif /* BOOZ_CONTROL_H */
