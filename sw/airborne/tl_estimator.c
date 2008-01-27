#include "tl_estimator.h"
#include "gps.h"
#include "flight_plan.h"

bool_t estimator_in_flight;

float estimator_east; /* m */
float estimator_north; /* m */
float estimator_z; /* m */

float estimator_speed; /* m/s */
float estimator_climb; /* m/s */
float estimator_course; /* rad, CCW */

void tl_estimator_init(void) {
}

void tl_estimator_use_gps(void) {
  float gps_east = gps_utm_east / 100.;
  float gps_north = gps_utm_north / 100.;

  /* Relative position to reference */
  estimator_east = gps_east - NAV_UTM_EAST0;
  estimator_north = gps_north - NAV_UTM_NORTH0;
  estimator_z = gps_alt / 100.;

  estimator_speed = gps_gspeed / 100.;
  estimator_climb = gps_climb / 100.;
  estimator_course = RadOfDeg(gps_course / 10.);
}
