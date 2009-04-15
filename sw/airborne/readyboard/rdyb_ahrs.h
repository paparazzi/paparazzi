#ifndef RDYB_AHRS_H
#define RDYB_AHRS_H

#include "rdyb_algebra.h"

struct RdybAhrs {
  int status;
  struct FloatQuat  ltp2imu_quat;
  struct FloatRMat  ltp2imu_rmat;
  struct FloatEuler ltp2imu_euler;
  struct FloatVect3 bias;
  struct FloatVect3 imu_rate;
 
  struct FloatEuler measurement;
 
};


extern struct RdybAhrs* ahrs_new(void);
extern void ahrs_propagate(struct RdybAhrs* me, struct FloatVect3* gyro);
extern void ahrs_update(struct RdybAhrs* me, struct FloatVect3* accel, struct FloatVect3* mag);

#endif /* RDYB_AHRS_H */
