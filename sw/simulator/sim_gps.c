/* OCaml binding to link the simulator to autopilot functions. */

#include <assert.h>
#include <inttypes.h>
#include "airframe.h"
#include "flight_plan.h"
#include "autopilot.h"
#include "gps.h"

#include <caml/mlvalues.h>

uint8_t gps_mode;
float   gps_ftow;    /* ms */
float   gps_falt;    /* m       */
float   gps_fspeed;  /* m/s     */
float   gps_fclimb;  /* m/s     */
float   gps_fcourse; /* rad     */
int32_t gps_utm_east, gps_utm_north;
uint8_t gps_utm_zone;
float gps_east, gps_north; /* m */
struct svinfo gps_svinfos[NB_CHANNELS];
uint8_t gps_nb_channels = 0;

value sim_use_gps_pos(value x, value y, value z, value c, value a, value s, value cl, value t) {
  gps_mode = 3;
  gps_utm_east = Int_val(x);
  gps_utm_north = Int_val(y);
  gps_utm_zone = Int_val(z);
  gps_fcourse = Double_val(c);
  gps_falt = Double_val(a);
  gps_fspeed = Double_val(s);
  gps_fclimb = Double_val(cl);
  gps_ftow = Double_val(t);

  gps_east = gps_utm_east / 100 - NAV_UTM_EAST0;
  gps_north = gps_utm_north / 100 - NAV_UTM_NORTH0;

  /** Space vehicle info simulation */
  gps_nb_channels=7;
  int i;
  static int time;
  time++;
  for(i = 0; i < gps_nb_channels; i++) {
    gps_svinfos[i].svid = 7 + i;
    gps_svinfos[i].elev = (cos(((100*i)+time)/100.) + 1) * 45;
    gps_svinfos[i].azim = (time/gps_nb_channels + 20 * i) % 360;
    gps_svinfos[i].cno = 40 + sin(time/100.) * 10.;
    gps_svinfos[i].flags = 0x01;
    gps_svinfos[i].qi = (int)((time / 1000.) + i) % 8;
  }
      
  use_gps_pos(); /* From main.c */
  return Val_unit;
}

/* Second binding required because number of args > 5 */
value sim_use_gps_pos_bytecode(value *a, int argn) {
  assert(argn == 8);
  return sim_use_gps_pos(a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7]);
}
