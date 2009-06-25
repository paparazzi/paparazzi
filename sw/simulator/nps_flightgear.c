#include "nps_flightgear.h"

#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <math.h>

#include "std.h"

#define FG_NET_GUI_VERSION 7
#define FG_NET_GUI_MAX_TANKS 4
struct FGNetGUI {
  uint32_t version;           // increment when data values change

  // Positions
  double longitude;           // geodetic (radians)
  double latitude;            // geodetic (radians)
  float altitude;             // above sea level (meters)
  float agl;                  // above ground level (meters)
  float phi;                  // roll (radians)
  float theta;                // pitch (radians)
  float psi;                  // yaw or true heading (radians)
  
  // Velocities
  float vcas;
  float climb_rate;           // feet per second

  // Consumables
  uint32_t num_tanks;         // Max number of fuel tanks
  float fuel_quantity[FG_NET_GUI_MAX_TANKS];

  // Environment
  uint32_t cur_time;          // current unix time
                              // FIXME: make this uint64_t before 2038
  uint32_t warp;              // offset in seconds to unix time
  float ground_elev;          // ground elev (meters)

  // Approach
  float tuned_freq;           // currently tuned frequency
  float nav_radial;           // target nav radial
  uint32_t in_range;           // tuned navaid is in range?
  float dist_nm;              // distance to tuned navaid in nautical miles
  float course_deviation_deg; // degrees off target course
  float gs_deviation_deg;     // degrees off target glide slope
};




#include "nps_fdm.h"

static struct  {
  int socket;
  struct sockaddr_in addr;


} flightgear;


void nps_flightgear_init(const char* host,  unsigned int port) {
  int so_reuseaddr = 1;
  struct protoent * pte = getprotobyname("UDP");
  flightgear.socket = socket( PF_INET, SOCK_DGRAM, pte->p_proto);
  setsockopt(flightgear.socket, SOL_SOCKET, SO_REUSEADDR, 
	     &so_reuseaddr, sizeof(so_reuseaddr));
  flightgear.addr.sin_family = PF_INET;
  flightgear.addr.sin_port = htons(port);
  flightgear.addr.sin_addr.s_addr = inet_addr(host);
}

void nps_flightgear_send() {
  
  struct FGNetGUI gui;
  //  net_gui_init(&gui);

  gui.version = FG_NET_GUI_VERSION; 

#if 0
  gui.latitude  = fdm.lla_pos.lat;
  gui.longitude = fdm.lla_pos.lon;
  gui.altitude  = fdm.lla_pos.alt;
#else
  gui.latitude = 0.656480;
  gui.longitude = -2.135537;
  gui.altitude = 0.807609;
#endif

  gui.agl = 1.111652;
  
  gui.phi = 0.;
  gui.theta = 0.;
  gui.psi = 5.20;
  
  gui.vcas = 0.;
  gui.climb_rate = 0.;

  gui.num_tanks = 1;
  gui.fuel_quantity[0] = 0.;

  gui.cur_time = 3198060679ul;
  gui.warp = 1122474394ul;

  gui.ground_elev = 0.;

  gui.tuned_freq = 125.65;
  gui.nav_radial = 90.;
  gui.in_range = 1;
  gui.dist_nm = 10.;
  gui.course_deviation_deg = 0.;
  gui.gs_deviation_deg = 0.;


  if (sendto(flightgear.socket, (char*)(&gui), sizeof(gui), 0,
             (struct sockaddr*)&flightgear.addr, sizeof(flightgear.addr)) == -1)
    printf("error sending\n");

}
