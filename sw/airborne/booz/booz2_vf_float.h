#ifndef BOOZ2_VF_FLOAT_H
#define BOOZ2_VF_FLOAT_H

#define B2_VFF_STATE_SIZE 3

extern float b2_vff_z;
extern float b2_vff_zdot;
extern float b2_vff_bias;
extern float b2_vff_P[B2_VFF_STATE_SIZE][B2_VFF_STATE_SIZE];
extern float b2_vff_zdotdot;

extern float b2_vff_z_meas;

extern void b2_vff_init(float z, float zdot, float bias);
extern void b2_vff_propagate(float accel);
extern void b2_vff_update(float z_meas);
extern void b2_vff_realign(float z_meas);

#endif /* BOOZ2_VF_FLOAT_H */

