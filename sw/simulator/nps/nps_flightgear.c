#include "nps_flightgear.h"

#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <math.h>


#include "std.h"
#include "../flight_gear.h"
#include "nps_fdm.h"

static struct  {
  int socket;
  struct sockaddr_in addr;
  unsigned int initial_time;
  unsigned int time_offset;
} flightgear;


void nps_flightgear_init(const char* host,  unsigned int port, unsigned int time_offset) {
  int so_reuseaddr = 1;
  struct protoent * pte = getprotobyname("UDP");
  flightgear.socket = socket( PF_INET, SOCK_DGRAM, pte->p_proto);
  setsockopt(flightgear.socket, SOL_SOCKET, SO_REUSEADDR,
         &so_reuseaddr, sizeof(so_reuseaddr));
  flightgear.addr.sin_family = PF_INET;
  flightgear.addr.sin_port = htons(port);
  flightgear.addr.sin_addr.s_addr = inet_addr(host);

  // get current time to use as inital when computing cur_time for FG
  //struct timeval t;
  //gettimeofday(&t, NULL);
  time_t t = time(NULL);
  //struct tm tm;
  //localtime_r(t, &tm);
  flightgear.initial_time = t;
  flightgear.time_offset = time_offset;
}

void nps_flightgear_send() {

  struct FGNetGUI gui;

  gui.version = FG_NET_GUI_VERSION;

  gui.latitude  = fdm.lla_pos.lat;
  gui.longitude = fdm.lla_pos.lon;
  gui.altitude  = fdm.lla_pos.alt;
  //  printf("%f %f %f\n", gui.latitude, gui.longitude, gui.altitude);

  gui.agl = 1.111652;

  gui.phi = fdm.ltp_to_body_eulers.phi;
  gui.theta = fdm.ltp_to_body_eulers.theta;
  gui.psi = fdm.ltp_to_body_eulers.psi;

  gui.vcas = 0.;
  gui.climb_rate = 0.;

  gui.num_tanks = 1;
  gui.fuel_quantity[0] = 0.;

  gui.cur_time = flightgear.initial_time + rint(fdm.time);
  // if cur_time is zero, flightgear would take the real current time
  //gui.cur_time = 0;
  // warp is used as an offset to the current time in seconds
  gui.warp = flightgear.time_offset;

  gui.ground_elev = 0.;

  gui.tuned_freq = 125.65;
  gui.nav_radial = 90.;
  gui.in_range = 1;
  gui.dist_nm = 10.;
  gui.course_deviation_deg = 0.;
  gui.gs_deviation_deg = 0.;


  if (sendto(flightgear.socket, (char*)(&gui), sizeof(gui), 0,
             (struct sockaddr*)&flightgear.addr, sizeof(flightgear.addr)) == -1) {
    fprintf(stderr, "error sending to FlightGear\n");
    fflush(stderr);
  }

}
