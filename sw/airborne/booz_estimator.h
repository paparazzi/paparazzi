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

#ifndef DISABLE_NAV

extern float booz_estimator_dcm[AXIS_NB][AXIS_NB];

/* position in earth frame : not yet available - sim only */
extern float booz_estimator_x;
extern float booz_estimator_y;
extern float booz_estimator_z;

/* speed in earth frame : not yet available - sim only */
extern float booz_estimator_vx;
extern float booz_estimator_vy;
extern float booz_estimator_vz;
#endif /* DISABLE_NAV */


extern void booz_estimator_init( void );
extern void booz_estimator_read_inter_mcu_state( void );


#ifndef DISABLE_NAV
extern void booz_estimator_compute_dcm( void );
extern void booz_estimator_set_speed_and_pos(float _vx, float _vy, float _vz, float _x, float _y, float _z);
#endif

#endif /* BOOZ_ESTIMATOR_H */
