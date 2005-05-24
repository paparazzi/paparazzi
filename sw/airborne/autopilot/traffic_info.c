/* Informations relative to the other aircrafts */

#include "flight_plan.h"

struct ac_info_ {float east, north, heading, alt;};

struct ac_info_ the_other;

void set_the_other(float utm_x, float utm_y, float heading, float alt) {
  the_other.east = utm_x -  NAV_UTM_EAST0;
  the_other.north = utm_y - NAV_UTM_NORTH0;
  the_other.heading = heading;
  the_other.alt = alt;
}

struct ac_info_ * get_the_other(void) {
  return &the_other;
}
