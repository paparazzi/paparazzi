#ifndef AGL_VFILTER_H
#define AGL_VFILTER_H

#include "adc.h"

#define STATE_SIZE 3

extern float agl_vf_z;
extern float agl_vf_zdot;
extern float agl_vf_bias;
extern float agl_vf_P[STATE_SIZE][STATE_SIZE];

extern struct adc_buf agl_adc_buf;

extern float agl_vf_z_meas;

extern void agl_vf_init(float z, float zdot, float bias);
extern void agl_vf_predict(float accel);
extern void agl_vf_update(float z_meas);

#endif /* AGL_VFILTER_H */
