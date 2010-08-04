#ifndef OVERO_ESTIMATOR_H
#define OVERO_ESTIMATOR_H

#include <inttypes.h>

struct OveroEstimator {

  float azimuth;
  float elevation;
  float tilt;
  
  float tilt_dot;

};


extern struct OveroEstimator estimator;

extern void estimator_init(void);
extern void estimator_run(uint16_t tilt_measure);

#endif /* OVERO_CONTROLLER_H */
