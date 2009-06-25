#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include <sys/time.h>
#include <getopt.h>

#include "nps_fdm.h"
#include "nps_sensors.h"
#include "nps_atmosphere.h"
#include "nps_autopilot.h"
#include "nps_ivy.h"
#include "nps_flightgear.h"

#define SIM_DT     (1./512.)
#define DISPLAY_DT (1./25.)
#define HOST_TIMEOUT_MS 40
#define HOST_TIME_FACTOR 1.

static struct {
  struct timeval host_time_start;
  double host_time_factor;
  double sim_time;
  double display_time;
  char* fg_host;
  unsigned int fg_port;
  char* joystick_dev;
} nps_main;

static bool_t nps_main_parse_options(int argc, char** argv);
static void nps_main_init(void);
static void nps_main_display(void);
static void nps_main_run_sim_step(void);
static gboolean nps_main_periodic(gpointer data __attribute__ ((unused)));


int main ( int argc, char** argv) {

  if (!nps_main_parse_options(argc, argv)) return 1;

  nps_main_init();
 
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  g_timeout_add(HOST_TIMEOUT_MS, nps_main_periodic, NULL);
  g_main_loop_run(ml);
  
  return 0;
}


static void nps_main_init(void) {

  nps_main.sim_time = 0.;
  nps_main.display_time = 0.;
  gettimeofday (&nps_main.host_time_start, NULL);
  nps_main.host_time_factor = HOST_TIME_FACTOR;

  nps_ivy_init();
  nps_fdm_init(SIM_DT);
  nps_sensors_init(nps_main.sim_time);
  nps_autopilot_init();

  if (nps_main.fg_host)
    nps_flightgear_init(nps_main.fg_host, nps_main.fg_port);

}



static void nps_main_run_sim_step(void) {


  //  printf("sim at %f\n", nps_main.sim_time);

  nps_fdm_run_step(autopilot.commands);

  nps_sensors_run_step(nps_main.sim_time);

  nps_autopilot_run_step(nps_main.sim_time);

}


static void nps_main_display(void) {
  //  printf("display at %f\n", nps_main.display_time);
  nps_ivy_display();
  if (nps_main.fg_host)
    nps_flightgear_send();
}


static gboolean nps_main_periodic(gpointer data __attribute__ ((unused))) {
  
  struct timeval host_time_now;
  gettimeofday (&host_time_now, NULL);
  double host_time_elapsed = nps_main.host_time_factor *
    ((host_time_now.tv_sec  - nps_main.host_time_start.tv_sec) +
     (host_time_now.tv_usec - nps_main.host_time_start.tv_usec)*1e-6);

  while (nps_main.sim_time <= host_time_elapsed) {
    nps_main_run_sim_step();
    nps_main.sim_time += SIM_DT;
    if (nps_main.display_time < nps_main.sim_time) {
      nps_main_display();
      nps_main.display_time += DISPLAY_DT;
    }
  }  
  
  return TRUE;

}


static bool_t nps_main_parse_options(int argc, char** argv) {

  nps_main.fg_host = NULL;
  nps_main.fg_port = 5501;
  nps_main.joystick_dev = NULL;

  static const char* usage =
"Usage: %s [options]\n"
" Options :\n"
"   -j --js_dev joystick device\n"
"   --fg_host flight gear host\n"
"   --fg_port flight gear port\n";


  while (1) {

    static struct option long_options[] = {
      {"fg_host", 1, NULL, 0},
      {"fg_port", 1, NULL, 0},
      {"js_dev", 1, NULL, 0},
      {0, 0, 0, 0}
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "j:",
			long_options, &option_index);
    if (c == -1)
      break;
    
    switch (c) {
    case 0:
      switch (option_index) {
      case 0:
	nps_main.fg_host = strdup(optarg); break;
      case 1:
	nps_main.fg_port = atoi(optarg); break;
      case 2:
	nps_main.joystick_dev = strdup(optarg); break;
      }
      break;

    case 'j':
      nps_main.joystick_dev = strdup(optarg);
      break;
    
    default: /* ’?’ */
      printf("?? getopt returned character code 0%o ??\n", c);
      fprintf(stderr, usage, argv[0]);
      exit(EXIT_FAILURE);
    }
  }
  return TRUE;
}
