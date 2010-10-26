/* OCaml binding to link the simulator to autopilot functions. */

#include <assert.h>
#include <math.h>
#include <inttypes.h>

/** From airborne/autopilot/ */
#include "airframe.h"
#include "flight_plan.h"
#include "autopilot.h"
#include "gps.h"
#include "estimator.h"
#include "latlong.h"
#include "common_nav.h"

uint8_t gps_mode;
uint16_t  gps_week;    /* weeks */
uint32_t  gps_itow;     /* ms */
int32_t   gps_alt;      /* cm       */
uint16_t  gps_gspeed;   /* cm/s     */
int16_t   gps_climb;    /* cm/s     */
int16_t   gps_course;   /* decideg     */
int32_t gps_utm_east, gps_utm_north;
uint8_t gps_utm_zone;
int32_t gps_lat, gps_lon;
struct svinfo gps_svinfos[GPS_NB_CHANNELS];
uint8_t gps_nb_channels = 0;
uint16_t gps_PDOP;
uint32_t gps_Pacc, gps_Sacc;
uint8_t gps_numSV;
uint16_t gps_reset;


void sim_use_gps_pos(double lat, double lon, double alt, double course, double gspeed, double climb, double time) {
    
  gps_mode = 3; // Mode 3D
  gps_course = DegOfRad(course) * 10.;
  gps_alt = alt * 100.;
  gps_gspeed = gspeed * 100.;
  gps_climb = climb * 100.;
  gps_week = 0; // FIXME
  gps_itow = time * 1000.;

  gps_lat = DegOfRad(lat)*1e7;
  gps_lon = DegOfRad(lon)*1e7;
  latlong_utm_of(lat, lon, nav_utm_zone0);
  gps_utm_east = latlong_utm_x * 100;
  gps_utm_north = latlong_utm_y * 100;
  gps_utm_zone = nav_utm_zone0;

}

/** Space vehicle info simulation */
void sim_update_sv(void) {

  gps_nb_channels=7;
  int i;
  static int time;
  time++;
  for(i = 0; i < gps_nb_channels; i++) {
    gps_svinfos[i].svid = 7 + i;
    gps_svinfos[i].elev = (cos(((100*i)+time)/100.) + 1) * 45;
    gps_svinfos[i].azim = (time/gps_nb_channels + 50 * i) % 360;
    gps_svinfos[i].cno = 40 + sin((time+i*10)/100.) * 10.;
    gps_svinfos[i].flags = ((time/10) % (i+1) == 0 ? 0x00 : 0x01);
    gps_svinfos[i].qi = (int)((time / 1000.) + i) % 8;
  }
  gps_PDOP = gps_Sacc = gps_Pacc = 500+200*sin(time/100.);
  gps_numSV = 7;
      
  gps_verbose_downlink = !launch;
  UseGpsPosNoSend(estimator_update_state_gps);
  gps_downlink();

}

void ubxsend_cfg_rst(uint16_t a __attribute__ ((unused)), uint8_t b __attribute__ ((unused))) {
  return;
}

