/* Datalink parsing functions */

#include "jsbsim_hw.h"

#include "math/pprz_geodetic_float.h"

#include <stdlib.h>

#define MOfCm(_x) (((float)(_x))/100.)

void parse_dl_ping(char* argv[] __attribute__ ((unused))) {
  DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
}

void parse_dl_acinfo(char* argv[] __attribute__ ((unused))) {
#ifdef TRAFFIC_INFO
  uint8_t id = atoi(argv[8]);
  float ux = MOfCm(atoi(argv[2]));
  float uy = MOfCm(atoi(argv[3]));
  float a = MOfCm(atoi(argv[4]));
  float c = RadOfDeg(((float)atoi(argv[1]))/ 10.);
  float s = MOfCm(atoi(argv[6]));
  float cl = MOfCm(atoi(argv[7]));
  uint32_t t = atoi(argv[5]);
  SetAcInfo(id, ux, uy, c, a, s, cl, t);
#endif
}

void parse_dl_setting(char* argv[]) {
  uint8_t index = atoi(argv[2]);
  float value = atof(argv[3]);
  DlSetting(index, value);
  DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice,&index, &value);
}

void parse_dl_get_setting(char* argv[]) {
  uint8_t index = atoi(argv[2]);
  float value = settings_get_value(index);
  DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice,&index, &value);
}

void parse_dl_block(char* argv[]) {
  int block = atoi(argv[1]);
  nav_goto_block(block);
}

void parse_dl_move_wp(char* argv[]) {
  uint8_t wp_id = atoi(argv[1]);
  float a = MOfCm(atoi(argv[5]));

  /* Computes from (lat, long) in the referenced UTM zone */
  struct LlaCoor_f lla;
  lla.lat = RadOfDeg((float)(atoi(argv[3]) / 1e7));
  lla.lon = RadOfDeg((float)(atoi(argv[4]) / 1e7));
  struct UtmCoor_f utm;
  utm.zone = nav_utm_zone0;
  utm_of_lla_f(&utm, &lla);
  nav_move_waypoint(wp_id, utm.east, utm.north, a);

  /* Waypoint range is limited. Computes the UTM pos back from the relative
     coordinates */
  utm.east = waypoints[wp_id].x + nav_utm_east0;
  utm.north = waypoints[wp_id].y + nav_utm_north0;
  DOWNLINK_SEND_WP_MOVED(DefaultChannel, DefaultDevice, &wp_id, &utm.east, &utm.north, &a, &nav_utm_zone0);
}

