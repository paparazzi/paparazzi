#include "booz_flightgear.h"
#include "flight_gear.h"

#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static int fg_socket;
static struct sockaddr_in fg_addr;

void net_gui_init (struct FGNetGUI* gui);

void booz_flightgear_init(const char* host,  unsigned int port) {

  printf("connecting to %s on port %d\n", host, port);

  int so_reuseaddr = 1;
  struct protoent * pte = getprotobyname("UDP");
  fg_socket = socket( PF_INET, SOCK_DGRAM, pte->p_proto);
  setsockopt(fg_socket, SOL_SOCKET, SO_REUSEADDR, 
	     &so_reuseaddr, sizeof(so_reuseaddr));
  
  fg_addr.sin_family = PF_INET;
  fg_addr.sin_port = htons(port);
  fg_addr.sin_addr.s_addr = inet_addr(host);

}


void booz_flightgear_send() {

  const double earth_radius = 6372795.;

  double lat =  0.656480 + asin((bfm.state->ve[BFMS_X] - 90)/earth_radius);
  double lon = -2.135537 + asin((bfm.state->ve[BFMS_Y] - 45)/earth_radius);

  struct FGNetGUI gui;
  net_gui_init(&gui);

  gui.latitude = lat;
  gui.longitude = lon;
  gui.altitude = 1.1 - bfm.state->ve[BFMS_Z];  

  gui.phi = bfm.state->ve[BFMS_PHI];
  gui.theta = bfm.state->ve[BFMS_THETA];
  gui.psi = bfm.state->ve[BFMS_PSI];

  gui.cur_time += (unsigned long)bfm.time;

  if (sendto(fg_socket, (char*)(&gui), sizeof(gui), 0,
             (struct sockaddr*)&fg_addr, sizeof(fg_addr)) == -1)
    printf("error sending\n");
}

void net_gui_init (struct FGNetGUI* gui) {
  gui->version = FG_NET_GUI_VERSION; 
  gui->latitude = 0.656480;
  gui->longitude = -2.135537;
  gui->altitude = 0.807609;
  gui->agl = 1.111652;
  
  gui->phi = 0.;
  gui->theta = 0.;
  gui->psi = 5.20;
  
  gui->vcas = 0.;
  gui->climb_rate = 0.;

  gui->num_tanks = 1;
  gui->fuel_quantity[0] = 0.;

  gui->cur_time = 3198060679ul;
  gui->warp = 1122474394ul;

  gui->ground_elev = 0.;

  gui->tuned_freq = 125.65;
  gui->nav_radial = 90.;
  gui->in_range = 1;
  gui->dist_nm = 10.;
  gui->course_deviation_deg = 0.;
  gui->gs_deviation_deg = 0.;
}
