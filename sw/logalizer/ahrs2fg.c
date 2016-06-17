#include <glib.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "flight_gear.h"
#include "network.h"

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

// Default Host
char default_fg_host[] = "127.0.0.1";
char* fg_host;
// Default TCP port
int fg_port = 5501;

#ifdef __APPLE__
char defaultIvyBus[] = "224.255.255.255:2010";
#else
char defaultIvyBus[] = "127.255.255.255:2010";
#endif
char* IvyBus;

int verbose = FALSE;

// quat int res
const float quat_int_res = (float)(1<<15);
// euler int res
const float euler_int_res = (float)(1<<12);

gfloat quat[4];
gfloat eulers[3];

gboolean timeout_callback(gpointer data) {

  static double x = 0.;
  static double y = 0.;
  static double z = 10.;
  const double earth_radius = 6372795.;

  double lat =  0.656480 + asin(x/earth_radius);
  double lon = -2.135537 + asin(y/earth_radius);

  struct FGNetGUI gui;
  net_gui_init(&gui);

  gui.latitude = lat;
  gui.longitude = lon;
  gui.altitude = z;

  if (verbose) {
    printf("phi:%f, theta:%f, psi:%f\n", eulers[0], eulers[1], eulers[2]);
    fflush(stdout);
  }
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

void on_ATTITUDE(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  float phi = atof(argv[0]);
  float psi = atof(argv[1]);
  float theta = atof(argv[2]);
  eulers[0] = phi;
  eulers[1] = theta;
  eulers[2] = psi;
}

void on_AHRS_EULER_INT(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  float phi = atof(argv[3]);
  float theta = atof(argv[4]);
  float psi = atof(argv[5]);
  eulers[0] = phi / euler_int_res;
  eulers[1] = theta / euler_int_res;
  eulers[2] = psi / euler_int_res;
}

void on_AHRS_QUAT_INT(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  float q0 = atof(argv[5]);
  float q1 = atof(argv[6]);
  float q2 = atof(argv[7]);
  float q3 = atof(argv[8]);
  quat[0] = q0 / quat_int_res;
  quat[1] = q1 / quat_int_res;
  quat[2] = q2 / quat_int_res;
  quat[3] = q3 / quat_int_res;
  quat_to_euler(quat, eulers);
}

void on_ROTORCRAFT_FP(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  float phi = atof(argv[6]);
  float theta = atof(argv[7]);
  float psi = atof(argv[8]);
  eulers[0] = phi / euler_int_res;
  eulers[1] = theta / euler_int_res;
  eulers[2] = psi / euler_int_res;
}

// Print help message
void print_help() {
  printf("Usage: ahrs2fg [options] <AC_ID>\n");
  printf(" Options :\n");
  printf("   -fg_host <host>\tFlight Gear host address (default: %s)\n", fg_host);
  printf("   -fg_port <port>\tFlight Gear port (default: %d)\n", fg_port);
  printf("   -b <Ivy bus>\tdefault is %s\n", defaultIvyBus);
  printf("   -v\tverbose\n");
  printf("   -h --help show this help\n");
}


int main ( int argc, char** argv) {

  int ac_id = -1;

  IvyBus = getenv("IVYBUS");
  if (IvyBus == NULL) IvyBus = defaultIvyBus;

  fg_host = default_fg_host;

  if (argc < 1) {
    print_help();
    exit(0);
  }

  // Parse options
  int i;
  for (i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      print_help();
      exit(0);
    }
    else if (strcmp(argv[i], "-fg_host") == 0) {
      fg_host = argv[++i];
    }
    else if (strcmp(argv[i], "-fg_port") == 0) {
      fg_port = atoi(argv[++i]);
    }
    else if (strcmp(argv[i], "-b") == 0) {
      IvyBus = argv[++i];
    }
    else if (strcmp(argv[i], "-v") == 0) {
      verbose = TRUE;
    }
    else {
      ac_id = atoi(argv[i]);
    }
  }
  if (ac_id == -1) {
    print_help();
    exit(0);
  }

  if (verbose) {
    printf("FG options: %s:%d\n", fg_host, fg_port);
    fflush(stdout);
  }
  struct FgNetChannel* chan = open_out_channel(fg_host, fg_port);

  g_timeout_add(50, timeout_callback, chan);

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("ahrs2fg", "ahrs2fg READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_ATTITUDE, chan, "^%d ATTITUDE (\\S*) (\\S*) (\\S*)", ac_id);
  IvyBindMsg(on_AHRS_EULER_INT, chan, "^%d AHRS_EULER_INT (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)", ac_id);
  IvyBindMsg(on_AHRS_QUAT_INT, chan, "^%d AHRS_QUAT_INT (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)", ac_id);
  IvyBindMsg(on_ROTORCRAFT_FP, chan, "^%d ROTORCRAFT_FP (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)", ac_id);
  if (verbose) {
    printf("Ivy options: %s\n", IvyBus);
    fflush(stdout);
  }
  IvyStart(IvyBus);

  g_main_loop_run(ml);
  return 0;
}
