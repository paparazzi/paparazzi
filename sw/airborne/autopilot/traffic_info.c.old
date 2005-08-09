/* Informations relative to the other aircrafts */

#include <inttypes.h>
#include "flight_plan.h"

#define NB_OTHERS 4

struct ac_info_ {float east, north, heading, alt;};

struct ac_info_ the_others[NB_OTHERS];

void
set_the_other(uint8_t id, float utm_x, float utm_y, float heading, float alt) {
  if (id < NB_OTHERS) {
    the_others[id].east = utm_x -  NAV_UTM_EAST0;
    the_others[id].north = utm_y - NAV_UTM_NORTH0;
    the_others[id].heading = heading;
    the_others[id].alt = alt;
  }
}

struct ac_info_ * get_the_other(uint8_t id) {
  id = (id < NB_OTHERS ? id : NB_OTHERS - 1);
  return &the_others[id];
}
