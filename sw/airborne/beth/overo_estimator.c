#include "overo_estimator.h"

#include "booz/booz_imu.h"

struct OveroEstimator estimator;

void estimator_init(void) {

}


void estimator_run(uint16_t tilt_measure) {
  
  const int32_t tilt_neutral = 2770;
  const float   tilt_scale = 1./580.;

  estimator.tilt = ((int32_t)tilt_measure - tilt_neutral) * tilt_scale;
  estimator.tilt_dot = booz_imu.gyro.q;


}

