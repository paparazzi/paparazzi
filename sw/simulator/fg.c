/** Values boxing for Flight Gear */

#include <string.h>
#include <caml/alloc.h>
#include <caml/mlvalues.h>
#include <caml/memory.h>

#include <math.h>
#include <time.h>

#include "flight_gear.h"

value fg_msg(value x, value y, value z, value phi) {
  CAMLparam4(x, y, z, phi);
  CAMLlocal1(s);

#if 0
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
  
  msg.cur_time = time(NULL);
  msg.warp = 0;
  msg.ground_elev = 0.;

  msg.tuned_freq = 123.45;
  msg.nav_radial = 123.;
  msg.in_range = 1;
  msg.course_deviation_deg = 12.;
  msg.gs_deviation_deg = 123.;
#endif

  static double _x = 0.;
  static double _y = 0.;
  static double _z = 10;

  const double v = 150.;
  const double psi = 292.80 * M_PI / 180.;
  
  const double vx = v * cos(psi);
  const double vy = v * sin(psi);
  const double vz = 1.;

  const double dt = 0.01666666;
  
  _x += vx * dt;
  _y += vy * dt;
  _z += vz * dt;

  const double earth_radius = 6372795.;

  double lat =  0.656480 + asin(_x/earth_radius);
  double lon = -2.135537 + asin(_y/earth_radius);

  struct FGNetGUI msg;
  //  net_gui_init(&gui);
  msg.version = FG_NET_GUI_VERSION; 
  //  gui.latitude = 0.656480;
  //  gui.longitude = -2.135537;
  //  gui.altitude = 0.807609;
  msg.agl = 1.111652;
  
  msg.phi = 0.;
  msg.theta = 0.;
  msg.psi = 5.20;
  
  msg.vcas = 0.;
  msg.climb_rate = 0.;

  msg.num_tanks = 1;
  msg.fuel_quantity[0] = 0.;

  msg.cur_time = 3198060679ul;
  msg.warp = 1122474394ul;

  msg.ground_elev = 0.;

  msg.tuned_freq = 125.65;
  msg.nav_radial = 90.;
  msg.in_range = 1;
  msg.dist_nm = 10.;
  msg.course_deviation_deg = 0.;
  msg.gs_deviation_deg = 0.;


  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = _z;

  s = alloc_string(sizeof(msg));
  strncpy(String_val(s), (char*)&msg, sizeof(msg));

  CAMLreturn (s);
}
