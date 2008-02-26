#ifndef TL_ESTIMATOR_H
#define TL_ESTIMATOR_H

#include "std.h"

extern bool_t estimator_in_flight;
extern bool_t tl_estimator_set_ground_pressure;
extern uint16_t estimator_flight_time;

extern float tl_estimator_u;
extern float tl_estimator_v;
extern float tl_estimator_u_dot;
extern float tl_estimator_v_dot;

extern float estimator_x; /* m */
extern float estimator_y; /* m */
extern float estimator_z; /* altitude in m */
extern float tl_estimator_agl; /* AGL in m */
extern float tl_estimator_agl_dot; /* AGL in m/s */


extern float estimator_speed; /* m/s */
extern float estimator_climb; /* m/s */
extern float estimator_course; /* rad, CCW */

extern float estimator_speed_east;
extern float estimator_speed_north;

extern float estimator_r;
extern float estimator_psi;  /* rad, CCW */ 
extern float estimator_z_baro;

extern float tl_estimator_cruise_power;

void tl_estimator_init(void);
void tl_estimator_use_gps(void);
void tl_estimator_use_gyro(void);
void tl_estimator_use_imu(void);
void tl_estimator_periodic_task(void);

extern float tl_baro_alpha;
extern float   tl_psi_kalm_psi;
extern float   tl_psi_kalm_bias;
extern float   tl_psi_kalm_P[2][2];

extern void tl_psi_kalm_init(void);
extern void tl_psi_kalm_start( float gyro, float angle);
extern void tl_psi_kalm_propagate( float gyro);
extern void tl_psi_kalm_update( float angle);


#define EstimatorSetAlt(_z) {estimator_z_baro = _z;}

void tl_estimator_to_body_frame(float east, float north, float *front, float *right);


#endif /* TL_ESTIMATOR_H */
