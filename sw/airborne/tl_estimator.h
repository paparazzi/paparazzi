#ifndef TL_ESTIMATOR_H
#define TL_ESTIMATOR_H

#include "std.h"

extern bool_t estimator_in_flight;
extern uint16_t estimator_flight_time;

extern float tl_estimator_u;
extern float tl_estimator_v;

extern float estimator_x; /* m */
extern float estimator_y; /* m */
extern float estimator_z; /* altitude in m */

extern float estimator_speed; /* m/s */
extern float estimator_climb; /* m/s */
extern float estimator_course; /* rad, CCW */

extern float estimator_r;
extern float estimator_psi;  /* rad, CCW */ 
extern float estimator_z_baro;

void tl_estimator_init(void);
void tl_estimator_use_gps(void);
void tl_estimator_use_gyro(void);
void tl_estimator_use_mag(void);


#define EstimatorSetAlt(_z) {estimator_z_baro = _z;}

void tl_estimator_to_body_frame(float east, float north, float *front, float *right);


#endif /* TL_ESTIMATOR_H */
