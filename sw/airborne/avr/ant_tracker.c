#include "ant_tracker.h"


uint8_t ant_track_mode;
float ant_track_azim;
float ant_track_elev;
uint8_t ant_track_id;

int32_t nav_utm_east0; 
int32_t nav_utm_north0;
uint8_t nav_utm_zone0;

void ant_tracker_init( void ) {
  //  nav_utm_east0 = ;
  //  nav_utm_north0 = ;
  //  nav_utm_zone0 = ;
  ant_track_id = 12;
  ant_track_mode = ANT_TRACK_AUTO;
  ant_track_azim = 0.;
  ant_track_elev = 0.;
}

void ant_tracker_periodic( void ) {
  if (ant_track_mode == ANT_TRACK_AUTO) {
    ant_track_azim += 0.5;
    if (ant_track_azim > 360.)
      ant_track_azim = 0.;
    ant_track_elev += 0.1;
    if (ant_track_elev > 90.)
      ant_track_elev = 0.;
  }
}
