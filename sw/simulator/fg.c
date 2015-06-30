/** Values boxing for Flight Gear */

#include <string.h>
#include <caml/alloc.h>
#include <caml/mlvalues.h>
#include <caml/memory.h>

#include <math.h>
#include <time.h>

#include "flight_gear.h"

value fg_sizeof(value unit) {
  return Val_int(sizeof(struct FGNetGUI));
}

value fg_msg_native(value s, value lat, value lon, value z, value phi, value theta, value psi);

value fg_msg_bytecode(value *argv, int argc) {
  return fg_msg_native(argv[0], argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);

}


value fg_msg_native(value s, value lat, value lon, value z, value phi, value theta, value psi) {

  struct FGNetGUI msg = {0};
  msg.version = FG_NET_GUI_VERSION;

  msg.longitude = Double_val(lon);
  msg.latitude = Double_val(lat);
  msg.altitude = Double_val(z) + 50;
  msg.agl = 0.;
  msg.phi = Double_val(phi);
  msg.theta = Double_val(theta);
  msg.psi = - Double_val(psi) + M_PI_2;

  msg.vcas = 0.;
  msg.climb_rate = 0.;

  msg.num_tanks = 1;
  msg.fuel_quantity[0] = 10.;

  msg.cur_time = 3213092700ul+((uint32_t)((msg.longitude)*13578)); //time(NULL);
  msg.warp = 0;
  msg.ground_elev = 0.;

  msg.tuned_freq = 123.45;
  msg.nav_radial = 123.;
  msg.in_range = 1;
  msg.course_deviation_deg = 12.;
  msg.gs_deviation_deg = 123.;

  memcpy(String_val(s), (char*)&msg, sizeof(msg));

  return Val_unit;
}
