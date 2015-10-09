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


double htond(double x)
{
  int *p = (int *)&x;
  int tmp = p[0];
  p[0] = htonl(p[1]);
  p[1] = htonl(tmp);

  return x;
}


float htonf(float x)
{
  int *p = (int *)&x;
  *p = htonl(*p);
  return x;
}


void nps_flightgear_init(const char *host,  unsigned int port, unsigned int time_offset)
{
  int so_reuseaddr = 1;
  struct protoent *pte = getprotobyname("UDP");
  flightgear.socket = socket(PF_INET, SOCK_DGRAM, pte->p_proto);
  setsockopt(flightgear.socket, SOL_SOCKET, SO_REUSEADDR,
             &so_reuseaddr, sizeof(so_reuseaddr));
  flightgear.addr.sin_family = PF_INET;
  flightgear.addr.sin_port = htons(port);
  flightgear.addr.sin_addr.s_addr = inet_addr(host);

  // get current time to use as inital when computing cur_time for FG
  time_t t = time(NULL);
  flightgear.initial_time = t;
  flightgear.time_offset = time_offset;
}

/**
 * Send FlightGear FDM packet
 * For visualization with moving surfaces (elevator, propeller etc).
 * start fgfs with --native-fdm=socket... option
 */
void nps_flightgear_send_fdm()
{
  struct FGNetFDM fgfdm;

  memset(&fgfdm, 0, sizeof(fgfdm));
  fgfdm.version = htonl(FG_NET_FDM_VERSION);

  fgfdm.latitude  = htond(fdm.lla_pos.lat);
  fgfdm.longitude = htond(fdm.lla_pos.lon);
  fgfdm.altitude  = htond(fdm.lla_pos.alt);


  fgfdm.agl = htonf((float)fdm.agl);

  fgfdm.phi = htonf((float)fdm.ltp_to_body_eulers.phi);
  fgfdm.theta = htonf((float)fdm.ltp_to_body_eulers.theta);
  fgfdm.psi = htonf((float)fdm.ltp_to_body_eulers.psi);

  fgfdm.vcas = htonf(0.);
  fgfdm.climb_rate = htonf(0.);

  fgfdm.num_tanks = htonl(1);
  fgfdm.fuel_quantity[0] = htonf(0.);

  fgfdm.cur_time = htonl(flightgear.initial_time + rint(fdm.time));
  // if cur_time is zero, flightgear would take the real current time
  //gui.cur_time = 0;
  // warp is used as an offset to the current time in seconds
  fgfdm.warp = htonl(flightgear.time_offset);

  // Engine
  fgfdm.num_engines = htonl(fdm.num_engines);
  for (int k = 0; k < FG_NET_FDM_MAX_ENGINES; k++) {
    // Temprary hack to clearly show when the engine is running
    if (fdm.eng_state[k] == 1) {
      fgfdm.rpm[k] = htonf(fdm.rpm[k]);
    } else {
      fgfdm.rpm[k] = htonf(0.0);
    }
    fgfdm.eng_state[k] = htonl(fdm.eng_state[k]);
  }

  //control surfaces
  fgfdm.elevator = htonf(fdm.elevator);
  fgfdm.left_aileron = htonf(fdm.left_aileron);
  fgfdm.right_aileron = htonf(fdm.right_aileron);
  fgfdm.rudder = htonf(fdm.rudder);
  fgfdm.left_flap = htonf(fdm.flap);
  fgfdm.right_flap = htonf(fdm.flap);

  if (sendto(flightgear.socket, (char *)(&fgfdm), sizeof(fgfdm), 0,
             (struct sockaddr *)&flightgear.addr, sizeof(flightgear.addr)) == -1) {
    fprintf(stderr, "error sending to FlightGear\n");
    fflush(stderr);
  }
}

/**
 * Send FlightGear GUI packet
 * For visualization of airplane position and attitude only
 * start fgfs with --native-gui=socket... option
 *
 * This is the default option
 */
void nps_flightgear_send()
{

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


  if (sendto(flightgear.socket, (char *)(&gui), sizeof(gui), 0,
             (struct sockaddr *)&flightgear.addr, sizeof(flightgear.addr)) == -1) {
    fprintf(stderr, "error sending to FlightGear\n");
    fflush(stderr);
  }

}
