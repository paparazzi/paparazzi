#ifndef BOOZ2_STABILIZATION_ATTITUDE_H
#define BOOZ2_STABILIZATION_ATTITUDE_H

#include "booz_geometry_int.h"

extern void booz2_stabilization_attitude_init(void);
extern void booz2_stabilization_attitude_read_rc(bool_t in_flight);
extern void booz2_stabilization_attitude_enter(void);
extern void booz2_stabilization_attitude_run(bool_t  in_flight);


//extern struct booz_ieuler booz_stabilization_att_sp;
//extern struct booz_ieuler booz_stabilization_att_ref;
//extern struct booz_ivect  booz_stabilization_rate_ref;
//extern struct booz_ivect  booz_stabilization_accel_ref;
#include "booz2_stabilization_attitude_ref_traj_euler.h"


extern struct booz_ivect  booz_stabilization_igain;
extern struct booz_ivect  booz_stabilization_pgain;
extern struct booz_ivect  booz_stabilization_dgain;
extern struct booz_ivect  booz_stabilization_ddgain;
extern struct booz_ieuler booz_stabilization_att_sum_err;

#endif /* BOOZ2_STABILIZATION_ATTITUDE_H */
