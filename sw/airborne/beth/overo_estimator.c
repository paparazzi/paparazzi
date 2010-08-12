#include "overo_estimator.h"

#include "booz/booz_imu.h"

struct OveroEstimator estimator;

void estimator_init(void) {

}

//zyx
void estimator_run(uint16_t tilt_measure, uint16_t elevation_measure, uint16_t azimuth_measure) {
  
  const int32_t tilt_neutral = 1970;
  const float   tilt_scale = 1./580.;
  const int32_t azimuth_neutral = 1500;
  const float   azimuth_scale = 1./580.;
  const int32_t elevation_neutral = 670;
  const float   elevation_scale = 1./580.;


  estimator.tilt = -(tilt_neutral - (int32_t)tilt_measure ) * tilt_scale;
  estimator.tilt_dot = RATE_FLOAT_OF_BFP(booz_imu.gyro.q);

  //TODO: Add rotation compensation to remove tilt effect on elev/azi
  estimator.elevation = (elevation_neutral - (int32_t)elevation_measure ) * elevation_scale;
  estimator.elevation_dot = RATE_FLOAT_OF_BFP(booz_imu.gyro.p);

  estimator.azimuth = (azimuth_neutral - (int32_t)azimuth_measure ) * azimuth_scale;
  estimator.azimuth_dot = RATE_FLOAT_OF_BFP(booz_imu.gyro.r);

}

