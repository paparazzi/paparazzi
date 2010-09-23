#include "ant_tracker.h"

#include "traffic_info.h"

uint8_t ant_track_mode;
float ant_track_azim;
float ant_track_elev;
uint8_t ant_track_id;

int32_t nav_utm_east0; 
int32_t nav_utm_north0;
uint8_t nav_utm_zone0;
const float ant_track_gnd_alt = 185.;

void ant_tracker_init( void ) {
  //  nav_utm_east0 = ;
  //  nav_utm_north0 = ;
  //  nav_utm_zone0 = ;
  ant_track_id = 5;
  ant_track_mode = ANT_TRACK_AUTO;
  ant_track_azim = 0.;
  ant_track_elev = 0.;
}

void ant_tracker_periodic( void ) {
  if (ant_track_mode == ANT_TRACK_AUTO) {
    struct ac_info_ * ac = get_ac_info(ant_track_id);
    ant_track_azim =  atan2(ac->north, ac->east) * 180. / M_PI;
    ant_track_azim = 90. - ant_track_azim;
    if (ant_track_azim < 0)
      ant_track_azim += 360.;
    float dist = sqrt(ac->north*ac->north + ac->east*ac->east);
    if ( dist < 1.) dist = 1.;
    float height = ac->alt - ant_track_elev;
    ant_track_elev =  atan2( height, dist) * 180. / M_PI;
  }
}
