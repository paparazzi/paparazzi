#include "tl_estimator.h"
#include "gps.h"
#include "flight_plan.h"

bool_t estimator_in_flight;
uint16_t estimator_flight_time;

float tl_estimator_u;
float tl_estimator_v;

float estimator_x; /* m */
float estimator_y; /* m */
float estimator_z; /* altitude in m */

float estimator_speed; /* m/s */
float estimator_climb; /* m/s */
float estimator_course; /* rad, CCW */

float estimator_psi; /* rad, CCW */ 

void tl_estimator_init(void) {
  tl_estimator_u = 0.;
  tl_estimator_v = 0.;
}

void tl_estimator_use_gps(void) {
  float gps_east = gps_utm_east / 100.;
  float gps_north = gps_utm_north / 100.;

  /* Relative position to reference */
  estimator_x = gps_east - NAV_UTM_EAST0;
  estimator_y = gps_north - NAV_UTM_NORTH0;
  estimator_z = gps_alt / 100.;

  estimator_speed = gps_gspeed / 100.;
  estimator_climb = gps_climb / 100.;
  estimator_course = RadOfDeg(gps_course / 10.);
}

void tl_estimator_to_body_frame(float east, float north,
				float *front, float *right) {
  float c = cos(estimator_psi);
  float s = sin(estimator_psi);
  
  *front = c * north + s * east;
  *right = - s * north + c * east;
}
