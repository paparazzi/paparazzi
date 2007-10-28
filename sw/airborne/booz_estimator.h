#ifndef BOOZ_ESTIMATOR_H
#define BOOZ_ESTIMATOR_H

extern float booz_estimator_uf_p;
extern float booz_estimator_uf_q;
extern float booz_estimator_uf_r;

extern float booz_estimator_p;
extern float booz_estimator_q;
extern float booz_estimator_r;

extern float booz_estimator_phi;
extern float booz_estimator_theta;
extern float booz_estimator_psi;

extern void booz_estimator_init( void );
extern void booz_estimator_read_inter_mcu_state( void );

#endif /* BOOZ_ESTIMATOR_H */
