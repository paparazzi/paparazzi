#ifndef BOOZ2_STABILIZATION_ATTITUDE_REF_TRAJ_EULER_H
#define BOOZ2_STABILIZATION_ATTITUDE_REF_TRAJ_EULER_H

#include "booz_geometry_mixed.h"

extern struct booz_ieuler booz_stabilization_att_sp;
extern struct booz_ieuler booz_stabilization_att_ref;
extern struct booz_ivect  booz_stabilization_rate_ref;
extern struct booz_ivect  booz_stabilization_accel_ref;

#define F_UPDATE_RES 9
#define F_UPDATE   (1<<F_UPDATE_RES)

#define ACCEL_REF_RES 12
#define ACCEL_REF_MAX_PQ (128*(1<<ACCEL_REF_RES))
#define ACCEL_REF_MAX_R  ( 32*(1<<ACCEL_REF_RES))

#define RATE_REF_RES  16
#define RATE_REF_MAX_PQ (  5*(1<<RATE_REF_RES))
#define RATE_REF_MAX_R  (  3*(1<<RATE_REF_RES))

#define ANGLE_REF_RES 20

#define PI_ANGLE_REF        (3.1415926535897932384626433832795029*(1<<ANGLE_REF_RES))
#define TWO_PI_ANGLE_REF (2.*3.1415926535897932384626433832795029*(1<<ANGLE_REF_RES))
#define ANGLE_REF_NORMALIZE(_a) {				\
    while (_a >  PI_ANGLE_REF)  _a -= TWO_PI_ANGLE_REF;		\
    while (_a < -PI_ANGLE_REF)  _a += TWO_PI_ANGLE_REF;		\
  }


#define OMEGA_PQ   RadOfDeg(800)
#define ZETA_PQ    0.85
#define ZETA_OMEGA_PQ_RES 10
#define ZETA_OMEGA_PQ BOOZ_INT_OF_FLOAT((ZETA_PQ*OMEGA_PQ), ZETA_OMEGA_PQ_RES)
#define OMEGA_2_PQ_RES 7
#define OMEGA_2_PQ    BOOZ_INT_OF_FLOAT((OMEGA_PQ*OMEGA_PQ), OMEGA_2_PQ_RES)
#define OMEGA_R   RadOfDeg(500)
#define ZETA_R    0.85
#define ZETA_OMEGA_R_RES 10
#define ZETA_OMEGA_R BOOZ_INT_OF_FLOAT((ZETA_R*OMEGA_R), ZETA_OMEGA_R_RES)
#define OMEGA_2_R_RES 7
#define OMEGA_2_R    BOOZ_INT_OF_FLOAT((OMEGA_R*OMEGA_R), OMEGA_2_R_RES)




#define BOOZ_STABILIZATION_ATTITUDE_REF_TRAJ_EULER_UPDATE() {		\
									\
    /* dumb integrate reference attitude        */			\
    const struct booz_ieuler d_angle = {				\
      booz_stabilization_rate_ref.x >> ( F_UPDATE_RES + RATE_REF_RES - ANGLE_REF_RES), \
      booz_stabilization_rate_ref.y >> ( F_UPDATE_RES + RATE_REF_RES - ANGLE_REF_RES), \
      booz_stabilization_rate_ref.z >> ( F_UPDATE_RES + RATE_REF_RES - ANGLE_REF_RES)};	\
    BOOZ_IEULER_SUM(booz_stabilization_att_ref, booz_stabilization_att_ref, d_angle ); \
    ANGLE_REF_NORMALIZE(booz_stabilization_att_ref.psi);		\
									\
    /* integrate reference rotational speeds   */			\
    const struct booz_ivect d_rate = {					\
      booz_stabilization_accel_ref.x >> ( F_UPDATE_RES + ACCEL_REF_RES - RATE_REF_RES), \
      booz_stabilization_accel_ref.y >> ( F_UPDATE_RES + ACCEL_REF_RES - RATE_REF_RES), \
      booz_stabilization_accel_ref.z >> ( F_UPDATE_RES + ACCEL_REF_RES - RATE_REF_RES)}; \
    BOOZ_IVECT_SUM(booz_stabilization_rate_ref, booz_stabilization_rate_ref, d_rate); \
    									\
    const struct booz_ivect MIN_RATE = { -RATE_REF_MAX_PQ, -RATE_REF_MAX_PQ, -RATE_REF_MAX_R }; \
    const struct booz_ivect MAX_RATE = {  RATE_REF_MAX_PQ,  RATE_REF_MAX_PQ,  RATE_REF_MAX_R }; \
    BOOZ_IVECT_BOUND(booz_stabilization_rate_ref, booz_stabilization_rate_ref, MIN_RATE, MAX_RATE); \
    									\
    /* compute reference attitude error        */			\
    struct booz_ieuler ref_err;						\
    BOOZ_IEULER_DIFF(ref_err, booz_stabilization_att_ref, booz_stabilization_att_sp); \
    /* wrap it in the shortest direction       */			\
    ANGLE_REF_NORMALIZE(ref_err.psi);					\
    									\
    /* compute reference angular accelerations */			\
    const struct booz_ivect accel_rate = {				\
      ((int32_t)(-2.*ZETA_OMEGA_PQ)* (booz_stabilization_rate_ref.x >> (RATE_REF_RES - ACCEL_REF_RES))) \
      >> (ZETA_OMEGA_PQ_RES),						\
      ((int32_t)(-2.*ZETA_OMEGA_PQ)* (booz_stabilization_rate_ref.y >> (RATE_REF_RES - ACCEL_REF_RES))) \
      >> (ZETA_OMEGA_PQ_RES),						\
      ((int32_t)(-2.*ZETA_OMEGA_R) * (booz_stabilization_rate_ref.z >> (RATE_REF_RES - ACCEL_REF_RES))) \
      >> (ZETA_OMEGA_R_RES) };						\
    									\
    const struct booz_ivect accel_angle = {				\
      ((int32_t)(-OMEGA_2_PQ)* (ref_err.phi   >> (ANGLE_REF_RES - ACCEL_REF_RES))) >> (OMEGA_2_PQ_RES), \
      ((int32_t)(-OMEGA_2_PQ)* (ref_err.theta >> (ANGLE_REF_RES - ACCEL_REF_RES))) >> (OMEGA_2_PQ_RES), \
      ((int32_t)(-OMEGA_2_R )* (ref_err.psi   >> (ANGLE_REF_RES - ACCEL_REF_RES))) >> (OMEGA_2_R_RES ) }; \
    									\
    BOOZ_IVECT_SUM(booz_stabilization_accel_ref, accel_rate, accel_angle); \
    									\
    const struct booz_ivect MIN_ACCEL = { -ACCEL_REF_MAX_PQ, -ACCEL_REF_MAX_PQ, -ACCEL_REF_MAX_R }; \
    const struct booz_ivect MAX_ACCEL = {  ACCEL_REF_MAX_PQ,  ACCEL_REF_MAX_PQ,  ACCEL_REF_MAX_R }; \
    BOOZ_IVECT_BOUND(booz_stabilization_accel_ref, booz_stabilization_accel_ref, MIN_ACCEL, MAX_ACCEL); \
    									\
    /* trim accel to zero if rate has been saturated */			\
    if (booz_stabilization_rate_ref.x >= RATE_REF_MAX_PQ) {		\
      booz_stabilization_rate_ref.x = RATE_REF_MAX_PQ;			\
      if (booz_stabilization_accel_ref.x > 0)				\
	booz_stabilization_accel_ref.x = 0;				\
    }									\
    else if (booz_stabilization_rate_ref.x <= -RATE_REF_MAX_PQ) {	\
      booz_stabilization_rate_ref.x = -RATE_REF_MAX_PQ;			\
      if (booz_stabilization_accel_ref.x < 0)				\
	booz_stabilization_accel_ref.x = 0;				\
    }									\
    if (booz_stabilization_rate_ref.y >= RATE_REF_MAX_PQ) {		\
      booz_stabilization_rate_ref.y = RATE_REF_MAX_PQ;			\
      if (booz_stabilization_accel_ref.y > 0)				\
	booz_stabilization_accel_ref.y = 0;				\
    }									\
    else if (booz_stabilization_rate_ref.y <= -RATE_REF_MAX_PQ) {	\
      booz_stabilization_rate_ref.y = -RATE_REF_MAX_PQ;			\
      if (booz_stabilization_accel_ref.y < 0)				\
	booz_stabilization_accel_ref.y = 0;				\
    }									\
    if (booz_stabilization_rate_ref.z >= RATE_REF_MAX_R) {		\
      booz_stabilization_rate_ref.z = RATE_REF_MAX_R;			\
      if (booz_stabilization_accel_ref.z > 0)				\
	booz_stabilization_accel_ref.z = 0;				\
    }									\
    else if (booz_stabilization_rate_ref.z <= -RATE_REF_MAX_R) {	\
      booz_stabilization_rate_ref.z = -RATE_REF_MAX_R;			\
      if (booz_stabilization_accel_ref.z < 0)				\
	booz_stabilization_accel_ref.z = 0;				\
    }									\
									\
  }



#define RC_UPDATE_FREQ 40

#define BOOZ2_STABILIZATION_ATTITUDE_READ_RC(_sp, _inflight) {		\
    									\
    _sp.phi =								\
      ((int32_t)-rc_values[RADIO_ROLL]  * BOOZ_STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ) \
      << (ANGLE_REF_RES - IANGLE_RES);					\
    _sp.theta =								\
      ((int32_t) rc_values[RADIO_PITCH] * BOOZ_STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ) \
      << (ANGLE_REF_RES - IANGLE_RES);					\
    if (_inflight) {							\
      if (rc_values[RADIO_YAW] >  BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R || \
	  rc_values[RADIO_YAW] < -BOOZ_STABILIZATION_ATTITUDE_DEADBAND_R ) { \
	_sp.psi +=							\
	  ((int32_t)-rc_values[RADIO_YAW] * BOOZ_STABILIZATION_ATTITUDE_SP_MAX_R / MAX_PPRZ / RC_UPDATE_FREQ) \
	  << (ANGLE_REF_RES - IANGLE_RES);				\
	ANGLE_REF_NORMALIZE(_sp.psi);					\
      }									\
    }									\
    else { /* if not flying, use current yaw as setpoint */		\
      _sp.psi = (booz_ahrs.ltp_to_body_euler.psi << (ANGLE_REF_RES - IANGLE_RES));		\
    }									\
  }

#define BOOZ2_STABILIZATION_ATTITUDE_ADD_SP(_add_sp) { \
    BOOZ_IEULER_SUM(booz_stabilization_att_sp,booz_stabilization_att_sp,_add_sp); \
    ANGLE_REF_NORMALIZE(booz_stabilization_att_sp.psi); \
}


#define BOOZ2_STABILIZATION_ATTITUDE_RESET_PSI_REF(_sp) {			\
    _sp.psi = booz_ahrs.ltp_to_body_euler.psi << (ANGLE_REF_RES - IANGLE_RES); \
    booz_stabilization_att_ref.psi = _sp.psi;				\
    booz_stabilization_rate_ref.z = 0;					\
  }


#endif /* BOOZ2_STABILIZATION_ATTITUDE_REF_TRAJ_EULER_H */
