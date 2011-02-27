/* OCaml binding to link the simulator to autopilot functions. */

#include <assert.h>
#include <math.h>
#include <inttypes.h>

/** From airborne/autopilot/ */
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include "subsystems/gps.h"
#include "estimator.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_int.h"

// currently needed to get nav_utm_zone0
#include "subsystems/navigation/common_nav.h"


void sim_use_gps_pos(double lat, double lon, double alt, double course, double gspeed, double climb, double time) {

  gps.fix = 3; // Mode 3D
  gps.course = course * 1e7;
  gps.hmsl = alt * 1000.;
  gps.gspeed = gspeed * 100.;
  gps.ned_vel.z = -climb * 100.;
  gps.week = 0; // FIXME
  gps.tow = time * 1000.;

  //TODO set alt above ellipsoid and hmsl

  struct LlaCoor_f lla_f;
  struct UtmCoor_f utm_f;
  lla_f.lat = lat;
  lla_f.lon = lon;
  utm_f.zone = nav_utm_zone0;
  utm_of_lla_f(&utm_f, &lla_f);
  LLA_BFP_OF_REAL(gps.lla_pos, lla_f);
  gps.utm_pos.east = utm_f.east*100;
  gps.utm_pos.north = utm_f.north*100;
  gps.utm_pos.zone = nav_utm_zone0;

  gps_available = TRUE;

}

/** Space vehicle info simulation */
void sim_update_sv(void) {

  gps.nb_channels=7;
  int i;
  static int time;
  time++;
  for(i = 0; i < gps.nb_channels; i++) {
    gps.svinfos[i].svid = 7 + i;
    gps.svinfos[i].elev = (cos(((100*i)+time)/100.) + 1) * 45;
    gps.svinfos[i].azim = (time/gps.nb_channels + 50 * i) % 360;
    gps.svinfos[i].cno = 40 + sin((time+i*10)/100.) * 10.;
    gps.svinfos[i].flags = ((time/10) % (i+1) == 0 ? 0x00 : 0x01);
    gps.svinfos[i].qi = (int)((time / 1000.) + i) % 8;
  }
  gps.pdop = gps.sacc = gps.pacc = 500+200*sin(time/100.);
  gps.num_sv = 7;

}

void ubxsend_cfg_rst(uint16_t a __attribute__ ((unused)), uint8_t b __attribute__ ((unused))) {
  return;
}

