/* OCaml binding to link the simulator to autopilot functions. */

#include <assert.h>
#include <math.h>
#include <inttypes.h>

/** From airborne/autopilot/ */
#include "airframe.h"
#include "flight_plan.h"
#include "autopilot.h"
#include "gps.h"

#include <caml/mlvalues.h>

uint8_t gps_mode;
uint32_t  gps_itow;    /* ms */
int32_t   gps_alt;    /* m       */
uint16_t  gps_gspeed;  /* cm/s     */
int16_t   gps_climb;  /* cm/s     */
int16_t   gps_course; /* decideg     */
int32_t gps_utm_east, gps_utm_north;
uint8_t gps_utm_zone;
struct svinfo gps_svinfos[NB_CHANNELS];
uint8_t gps_nb_channels = 0;

value sim_use_gps_pos(value x, value y, value z, value c, value a, value s, value cl, value t, value m) {
  gps_mode = (Bool_val(m) ? 3 : 0);
  gps_utm_east = Int_val(x);
  gps_utm_north = Int_val(y);
  gps_utm_zone = Int_val(z);
  gps_course = DegOfRad(Double_val(c)) * 10.;
  gps_alt = Double_val(a) * 100.;
  gps_gspeed = Double_val(s) * 100.;
  gps_climb = Double_val(cl) * 100.;
  gps_itow = Double_val(t) * 1000.;

  /** Space vehicle info simulation */
  gps_nb_channels=7;
  int i;
  static int time;
  time++;
  for(i = 0; i < gps_nb_channels; i++) {
    gps_svinfos[i].svid = 7 + i;
    gps_svinfos[i].elev = (cos(((100*i)+time)/100.) + 1) * 45;
    gps_svinfos[i].azim = (time/gps_nb_channels + 50 * i) % 360;
    gps_svinfos[i].cno = 40 + sin(time/100.) * 10.;
    gps_svinfos[i].flags = 0x01;
    gps_svinfos[i].qi = (int)((time / 1000.) + i) % 8;
  }
      
  use_gps_pos(); /* From main.c */
  return Val_unit;
}

/* Second binding required because number of args > 5 */
value sim_use_gps_pos_bytecode(value *a, int argn) {
  assert(argn == 9);
  return sim_use_gps_pos(a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7], a[8]);
}
