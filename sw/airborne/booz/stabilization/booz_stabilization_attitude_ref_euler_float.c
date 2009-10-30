#include "booz_stabilization.h"


struct FloatEulers booz_stab_att_sp_euler;
struct FloatEulers booz_stab_att_ref_euler;
struct FloatRates  booz_stab_att_ref_rate;
struct FloatRates  booz_stab_att_ref_accel;

void booz_stabilization_attitude_ref_init(void) {

  FLOAT_EULERS_ZERO(booz_stab_att_sp_euler);
  FLOAT_EULERS_ZERO(booz_stab_att_ref_euler);
  FLOAT_RATES_ZERO(booz_stab_att_ref_rate);
  FLOAT_RATES_ZERO(booz_stab_att_ref_accel);

}


/*
 * Reference
 */
#define DT_UPDATE (1./512.)

#define REF_ACCEL_MAX_P BOOZ_STABILIZATION_ATTITUDE_REF_MAX_PDOT
#define REF_ACCEL_MAX_Q BOOZ_STABILIZATION_ATTITUDE_REF_MAX_QDOT
#define REF_ACCEL_MAX_R BOOZ_STABILIZATION_ATTITUDE_REF_MAX_RDOT

#define REF_RATE_MAX_P BOOZ_STABILIZATION_ATTITUDE_REF_MAX_P
#define REF_RATE_MAX_Q BOOZ_STABILIZATION_ATTITUDE_REF_MAX_Q
#define REF_RATE_MAX_R BOOZ_STABILIZATION_ATTITUDE_REF_MAX_R

#define OMEGA_P   BOOZ_STABILIZATION_ATTITUDE_REF_OMEGA_P
#define OMEGA_Q   BOOZ_STABILIZATION_ATTITUDE_REF_OMEGA_Q
#define OMEGA_R   BOOZ_STABILIZATION_ATTITUDE_REF_OMEGA_R

#define ZETA_P    BOOZ_STABILIZATION_ATTITUDE_REF_ZETA_P
#define ZETA_Q    BOOZ_STABILIZATION_ATTITUDE_REF_ZETA_Q
#define ZETA_R    BOOZ_STABILIZATION_ATTITUDE_REF_ZETA_R


#define USE_REF 1

void booz_stabilization_attitude_ref_update() {

#ifdef USE_REF
  
    /* dumb integrate reference attitude        */			
    struct FloatRates delta_rate;
    RATES_SMUL(delta_rate, booz_stab_att_ref_rate, DT_UPDATE);
    struct FloatEulers delta_angle;
    EULERS_ASSIGN(delta_angle, delta_rate.p, delta_rate.q, delta_rate.r);
    EULERS_ADD(booz_stab_att_ref_euler, delta_angle );
    FLOAT_ANGLE_NORMALIZE(booz_stab_att_ref_euler.psi);
								
    /* integrate reference rotational speeds   */
    struct FloatRates delta_accel;
    RATES_SMUL(delta_accel, booz_stab_att_ref_accel, DT_UPDATE);
    RATES_ADD(booz_stab_att_ref_rate, delta_accel);

    /* compute reference attitude error        */
    struct FloatEulers ref_err;
    EULERS_DIFF(ref_err, booz_stab_att_ref_euler, booz_stab_att_sp_euler);
    /* wrap it in the shortest direction       */
    FLOAT_ANGLE_NORMALIZE(ref_err.psi);

    /* compute reference angular accelerations */
    booz_stab_att_ref_accel.p = -2.*ZETA_P*OMEGA_P*booz_stab_att_ref_rate.p - OMEGA_P*OMEGA_P*ref_err.phi;
    booz_stab_att_ref_accel.q = -2.*ZETA_Q*OMEGA_P*booz_stab_att_ref_rate.q - OMEGA_Q*OMEGA_Q*ref_err.theta;
    booz_stab_att_ref_accel.r = -2.*ZETA_R*OMEGA_P*booz_stab_att_ref_rate.r - OMEGA_R*OMEGA_R*ref_err.psi;

    /*	saturate acceleration */				
    const struct Int32Rates MIN_ACCEL = { -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
    const struct Int32Rates MAX_ACCEL = {  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R }; \
    RATES_BOUND_BOX(booz_stab_att_ref_accel, MIN_ACCEL, MAX_ACCEL);

    /* saturate speed and trim accel accordingly */
    SATURATE_SPEED_TRIM_ACCEL();
								
#else   /* !USE_REF */
  EULERS_COPY(booz_stabilization_att_ref, booz_stabilization_att_sp);
  FLOAT_RATES_ZERO(booz_stabilization_rate_ref);
  FLOAT_RATES_ZERO(booz_stabilization_accel_ref);
#endif /* USE_REF */


}


