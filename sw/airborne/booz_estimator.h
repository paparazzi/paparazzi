#ifndef BOOZ_ESTIMATOR_H
#define BOOZ_ESTIMATOR_H

#include "6dof.h"

/* unfiltered rates available when filter crashed or uninitialed */
extern float booz_estimator_uf_p;
extern float booz_estimator_uf_q;
extern float booz_estimator_uf_r;

extern float booz_estimator_p;
extern float booz_estimator_q;
extern float booz_estimator_r;

extern float booz_estimator_phi;
extern float booz_estimator_theta;
extern float booz_estimator_psi;

extern float booz_estimator_dcm[AXIS_NB][AXIS_NB];

/* position in earth frame : not yet available - sim only */
extern float booz_estimator_x;
extern float booz_estimator_y;
extern float booz_estimator_z;

/* speed in earth frame : not yet available - sim only */
extern float booz_estimator_vx;
extern float booz_estimator_vy;
extern float booz_estimator_vz;

/* speed in body frame : not yet available - sim only */
extern float booz_estimator_u;
extern float booz_estimator_v;
extern float booz_estimator_w;


extern void booz_estimator_init( void );
extern void booz_estimator_read_inter_mcu_state( void );


#endif /* BOOZ_ESTIMATOR_H */
