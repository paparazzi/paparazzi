#include "overo_estimator.h"

#include "booz/booz_imu.h"

struct OveroEstimator estimator;

void estimator_init(void) {

}


void estimator_run(uint16_t tilt_measure) {
  
  const int32_t tilt_neutral = 2815;
  const float   tilt_scale = 1./580.;
  //const float   tilt_scale = 1.;

  estimator.tilt = (tilt_neutral - (int32_t)tilt_measure ) * tilt_scale;
  //estimator.tilt = -(tilt_neutral - (int32_t)tilt_measure ) * tilt_scale;
  estimator.tilt_dot = RATE_FLOAT_OF_BFP(booz_imu.gyro.q);


}

