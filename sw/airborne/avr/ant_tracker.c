#include "ant_tracker.h"


uint8_t ant_track_mode;
float ant_track_azim;
float ant_track_elev;
uint8_t ant_track_id;

int32_t nav_utm_east0; 
int32_t nav_utm_north0;
uint8_t nav_utm_zone0;


void ant_tracker_update( void ) {
  if (ant_track_mode == ANT_TRACK_AUTO) {
    ant_track_azim += 0.5;
    if (ant_track_azim > 360.)
      ant_track_azim = 0.;
  }
}
