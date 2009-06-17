

#include "nps_fdm.h"
#include "nps_sensors.h"
#include "nps_athmosphere.h"
#include "nps_autopilot"


#define SIM_DT     (1./512.)
#define DISPLAY_DT (1./25.)


struct {
  double sim_time;
  double display_time;
} nps_main;


int main ( int argc, char** argv) {


  return 0;
}





void nps_main_init(void) {

  nps_main.sim_time = 0.;
  nps_main.display_time = 0.;

}



void nps_main_run_sim_step(void) {

  nps_fdm_run_step(autopilot.commands);

  nps_sensors_run_step();

  nps_autopilot_run_step();

}

void nps_main_display(void) {


}


gboolean nps_main_periodic(gpointer data __attribute__ ((unused))) {
  
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


