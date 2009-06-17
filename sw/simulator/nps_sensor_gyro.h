#ifndef NPS_SENSORS_GYRO_H
#define NPS_SENSORS_GYRO_H

#include <matrix.h>

struct NpsSensorsGyro {
  VEC*   gyro;
  unsigned int gyro_resolution;
  MAT*   gyro_sensitivity;
  VEC*   gyro_neutral;
  VEC*   gyro_noise_std_dev;
  VEC*   gyro_bias_initial;
  VEC*   gyro_bias_random_walk_std_dev;
  VEC*   gyro_bias_random_walk_value;
  double gyro_next_update;
  int    gyro_available;
}

#endif /* NPS_SENSORS_GYRO_H */

