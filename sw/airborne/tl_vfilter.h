#ifndef TL_VFILTER_H
#define TL_VFILTER_H

//#define FLOAT_T double
#define FLOAT_T float
#define STATE_SIZE 3

extern FLOAT_T tl_vf_z;
extern FLOAT_T tl_vf_zdot;
extern FLOAT_T tl_vf_bias;
extern FLOAT_T tl_vf_P[STATE_SIZE][STATE_SIZE];

extern FLOAT_T tl_vf_z_meas;

extern void tl_vf_init(FLOAT_T z, FLOAT_T zdot, FLOAT_T bias);
extern void tl_vf_predict(FLOAT_T accel);
extern void tl_vf_update(FLOAT_T z_meas);

#endif /* TL_VFILTER_H */
