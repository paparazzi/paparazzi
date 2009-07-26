#ifndef BOOZ_STABILIZATION_ATTITUDE_REF_H
#define BOOZ_STABILIZATION_ATTITUDE_REF_H

#define SATURATE_SPEED_TRIM_ACCEL() {				\
    if (booz_stab_att_ref_rate.p >= REF_RATE_MAX_P) {		\
      booz_stab_att_ref_rate.p = REF_RATE_MAX_P;		\
      if (booz_stab_att_ref_accel.p > 0)			\
	booz_stab_att_ref_accel.p = 0;				\
    }								\
    else if (booz_stab_att_ref_rate.p <= -REF_RATE_MAX_P) {	\
      booz_stab_att_ref_rate.p = -REF_RATE_MAX_P;		\
      if (booz_stab_att_ref_accel.p < 0)			\
	booz_stab_att_ref_accel.p = 0;				\
    }								\
    if (booz_stab_att_ref_rate.q >= REF_RATE_MAX_Q) {		\
      booz_stab_att_ref_rate.q = REF_RATE_MAX_Q;		\
      if (booz_stab_att_ref_accel.q > 0)			\
	booz_stab_att_ref_accel.q = 0;				\
    }								\
    else if (booz_stab_att_ref_rate.q <= -REF_RATE_MAX_Q) {	\
      booz_stab_att_ref_rate.q = -REF_RATE_MAX_Q;		\
      if (booz_stab_att_ref_accel.q < 0)			\
	booz_stab_att_ref_accel.q = 0;				\
    }								\
    if (booz_stab_att_ref_rate.r >= REF_RATE_MAX_R) {		\
      booz_stab_att_ref_rate.r = REF_RATE_MAX_R;		\
      if (booz_stab_att_ref_accel.r > 0)			\
	booz_stab_att_ref_accel.r = 0;				\
    }								\
    else if (booz_stab_att_ref_rate.r <= -REF_RATE_MAX_R) {	\
      booz_stab_att_ref_rate.r = -REF_RATE_MAX_R;		\
      if (booz_stab_att_ref_accel.r < 0)			\
	booz_stab_att_ref_accel.r = 0;				\
    }								\
  }

#endif /* BOOZ_STABILIZATION_ATTITUDE_REF_H */
