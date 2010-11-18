#ifndef OVERO_ESTIMATOR_H
#define OVERO_ESTIMATOR_H

#include <inttypes.h>

struct OveroEstimator {

  float azimuth;
  float elevation;
  float tilt;

  float azimuth_dot;
  float elevation_dot;
  float tilt_dot;

  float tilt_lp_coeff;
  float elevation_lp_coeff;
  float azimuth_lp_coeff;
};


extern struct OveroEstimator estimator;

extern void estimator_init(void);
extern void estimator_send_messages(void);
extern void estimator_run(uint16_t tilt_measure, uint16_t elevation_measure, uint16_t azimuth_measure);

#endif /* OVERO_CONTROLLER_H */
