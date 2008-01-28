#ifndef TL_ESTIMATOR_H
#define TL_ESTIMATOR_H

#include "std.h"

extern bool_t estimator_in_flight;
extern uint16_t estimator_flight_time;

extern float estimator_east; /* m */
extern float estimator_north; /* m */
extern float estimator_z; /* altitude in m */

extern float estimator_speed; /* m/s */
extern float estimator_climb; /* m/s */
extern float estimator_course; /* rad, CCW */


void tl_estimator_init(void);
void tl_estimator_use_gps(void);


#endif /* TL_ESTIMATOR_H */
