#include <glib.h>
#include <getopt.h>
#include <sys/time.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "nps_fdm.h"
#include "nps_jsbsim.h"

static struct NpsFdmState fdm_state;


/* rate of the host mainloop */
#define HOST_TIMEOUT_PERIOD 4
struct timeval host_time_start;
double host_time_elapsed;
double host_time_factor = 1.;

/* 250Hz <-> 4ms */
#define SIM_DT (1./10.)
double sim_time;


static gboolean sim_periodic(gpointer data __attribute__ ((unused))) {

  struct timeval host_time_now;
  gettimeofday (&host_time_now, NULL);
  host_time_elapsed = host_time_factor *
    ((host_time_now.tv_sec  - host_time_start.tv_sec) +
     (host_time_now.tv_usec - host_time_start.tv_usec)*1e-6);
  
  while (sim_time <= host_time_elapsed) {
    sim_run_one_step();
    sim_time += SIM_DT;
  }  
    
  return TRUE;
}

static void sim_run_one_step(void) {

  // run autopilot
  nps_jsbsim_feed_inputs(fdmex, &fdm_state);
  result = fdmex->Run();
  nps_jsbsim_fetch_state(fdmex, &fdm_state);
  
  // balancer les messages

  //  IvySendMsg("%d BOOZ_SIM_SPEED_POS %f %f %f %f %f %f",  
  //	     AC_ID,
  //	     (fdm_state.speed_ltp->ve[AXIS_X]), 
  //	     (bfm.speed_ltp->ve[AXIS_Y]), 
  //	     (bfm.speed_ltp->ve[AXIS_Z]),
  //         (bfm.pos_ltp->ve[AXIS_X]), 
  //	     (bfm.pos_ltp->ve[AXIS_Y]), 
  //	     (bfm.pos_ltp->ve[AXIS_Z]));


}

static void sim_init(void) {

  /* Setting JSBSim */
  
  if (~JSBInit(SIM_DT)) {/* message d'erreur */}
  
  /* Setting Ivy */

  gettimeofday (&host_time_start, NULL);
  sim_time = 0.;
  
  IvyInit ("nps_test2", "nps_test2 READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

}

int main ( int argc, char** argv) {

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(HOST_TIMEOUT_PERIOD, sim_periodic, NULL);

  g_main_loop_run(ml);
  
  return 0;
}
