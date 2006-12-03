#include <glib.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "flight_gear.h"
#include "network.h"

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

gfloat biases[3];
gfloat quat[4];
gfloat eulers[3];

gboolean timeout_callback(gpointer data) {

  static double x = 0.;
  static double y = 0.;
  static double z = 10;
  const double earth_radius = 6372795.;
  
  double lat =  0.656480 + asin(x/earth_radius);
  double lon = -2.135537 + asin(y/earth_radius);
  
  struct FGNetGUI gui;
  net_gui_init(&gui);

  gui.latitude = lat;
  gui.longitude = lon;
  gui.altitude = z;

  //  printf("%f %f %f\n", eulers[0], eulers[1], eulers[2]);
  gui.phi = eulers[0];
  gui.theta = eulers[1];
  gui.psi = eulers[2];

  //  net_gui_hton(&gui);
  send_buf((struct FgNetChannel*)data, (char*)&gui, sizeof(gui));
  return TRUE;
}

void quat_to_euler( gfloat* quat, gfloat* euler) {
  //  float q02 = quat[0] * quat[0];
  float q12 = quat[1] * quat[1];
  float q22 = quat[2] * quat[2];
  float q32 = quat[3] * quat[3];

  euler[0] = atan2( 2*(quat[2]*quat[3] + quat[0]*quat[1]),(1-2*(q12 + q22)) );
  euler[1] = -asin(2*(quat[1]*quat[3] - quat[0]*quat[2]));
  euler[2] = atan2( 2*(quat[1]*quat[2] + quat[0]*quat[3]),(1-2*(q22 + q32)) );
}

void on_AHRS_STATE(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  float q0 = atof(argv[0]);
  float q1 = atof(argv[1]);
  float q2 = atof(argv[2]);
  float q3 = atof(argv[3]);
  float bx = atof(argv[4]);
  float by = atof(argv[5]);
  float bz = atof(argv[6]);
  biases[0] = bx;
  biases[1] = by;
  biases[2] = bz;
  quat[0] = q0;
  quat[1] = q1;
  quat[2] = q2;
  quat[3] = q3;
  quat_to_euler(quat, eulers);
}


int main ( int argc, char** argv) {
  struct FgNetChannel* chan = open_out_channel("127.0.0.1", 5501);

  g_timeout_add(16, timeout_callback, chan);

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  IvyInit ("IvyGtkButton", "IvyGtkButton READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_AHRS_STATE, chan, "^77 AHRS_STATE (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  g_main_loop_run(ml);
  return 0;
}
