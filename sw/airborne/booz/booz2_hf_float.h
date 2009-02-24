#ifndef BOOZ2_HF_FLOAT_H
#define BOOZ2_HF_FLOAT_H

#include "pprz_algebra_float.h"
#include "pprz_algebra_int.h"

extern struct Int32Vect3 b2ins_accel_bias;
#define B2INS_ACCEL_BIAS_FRAC 19
extern struct Int32Vect3 b2ins_accel_ltp;
#define B2INS_ACCEL_LTP_FRAC 10
extern struct Int32Vect3 b2ins_speed_ltp;
#define B2INS_SPEED_LTP_FRAC 19
extern struct Int64Vect3 b2ins_pos_ltp;
#define B2INS_POS_LTP_FRAC   28

extern struct Int32Vect3  b2ins_meas_gps_pos_ned;
extern struct Int32Vect3  b2ins_meas_gps_speed_ned;

extern void b2ins_init(void);
extern void b2ins_propagate(void);
extern void b2ins_update_gps(void);

#endif /* BOOZ2_HF_FLOAT_H */
