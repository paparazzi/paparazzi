
#include <glib.h>

#include "nps_fdm.h"
#include "nps_sensors.h"
#include "nps_atmosphere.h"
#include "nps_autopilot.h"


#define SIM_DT     (1./512.)
#define DISPLAY_DT (1./25.)


static struct {
  double sim_time;
  double display_time;
} nps_main;

static void nps_main_init(void);
static void nps_main_display(void);
static void nps_main_run_sim_step(void);
static gboolean nps_main_periodic(gpointer data __attribute__ ((unused)));


int main ( int argc, char** argv) {

  nps_main_init();

  return 0;
}





static void nps_main_init(void) {

  nps_main.sim_time = 0.;
  nps_main.display_time = 0.;

  nps_fdm_init(SIM_DT);
  nps_sensors_init(nps_main.sim_time);
  nps_autopilot_init();

}



static void nps_main_run_sim_step(void) {

  nps_fdm_run_step(autopilot.commands);

  //  nps_sensors_run_step();

  //  nps_autopilot_run_step();

}


static void nps_main_display(void) {


}


static gboolean nps_main_periodic(gpointer data __attribute__ ((unused))) {
  
  /* FIXME */
#if 0
  struct timeval host_time_now;
  gettimeofday (&host_time_now, NULL);
  host_time_elapsed = host_time_factor *
    ((host_time_now.tv_sec  - host_time_start.tv_sec) +
     (host_time_now.tv_usec - host_time_start.tv_usec)*1e-6);
  
  while (sim_time <= host_time_elapsed) {
    nps_main_run_sim_step();
    sim_time += SIM_DT;
  }  
#endif

}


