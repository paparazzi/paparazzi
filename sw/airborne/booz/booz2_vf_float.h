#ifndef BOOZ2_VF_FLOAT_H
#define BOOZ2_VF_FLOAT_H

#define B2_VFF_STATE_SIZE 3

extern FLOAT_T b2_vff_z;
extern FLOAT_T b2_vff_zdot;
extern FLOAT_T b2_vff_bias;
extern FLOAT_T b2_vff_P[B2_VFF_STATE_SIZE][B2_VFF_STATE_SIZE];
extern FLOAT_T b2_vff_zdotdot;

extern FLOAT_T b2_vff_z_meas;

extern void b2_vff_init(FLOAT_T z, FLOAT_T zdot, FLOAT_T bias);
extern void b2_vff_propagate(FLOAT_T accel);
extern void b2_vff_update(FLOAT_T z_meas);

#endif /* BOOZ2_VF_FLOAT_H */

