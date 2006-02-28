/** Values boxing for Flight Gear */

#include <string.h>
#include <caml/alloc.h>
#include <caml/mlvalues.h>
#include <caml/memory.h>

#include "flight_gear.h"

value fg_msg(value x, value y, value z, value phi) {
  CAMLparam4(x, y, z, phi);
  CAMLlocal1(s);

  const double earth_radius = 6372795.; 
  double lat =  0.656480 + asin(Double_val(x)/earth_radius);
  double lon = -2.135537 + asin(Double_val(y)/earth_radius);
  
  struct FGNetGUI msg;
  msg.version = FG_NET_GUI_VERSION;

  msg.longitude = lon;
  msg.latitude = lat;
  msg.altitude = Double_val(z);
  msg.agl = 0.;
  msg.phi = Double_val(phi);
  msg.theta = 0.;
  msg.psi = 0.;

  msg.vcas = 0.;
  msg.climb_rate = 0.;
  
  msg.num_tanks = 1;
  msg.fuel_quantity[0] = 10.;
  
  msg.cur_time = 3213082700ul;
  msg.warp = 0;
  msg.ground_elev = 0.;

  msg.tuned_freq = 123.45;
  msg.nav_radial = 123.;
  msg.in_range = 1;
  msg.course_deviation_deg = 12.;
  msg.gs_deviation_deg = 123.;

  s = alloc_string(sizeof(msg));
  strncpy(String_val(s), (char*)&msg, sizeof(msg));

  CAMLreturn (s);
}
