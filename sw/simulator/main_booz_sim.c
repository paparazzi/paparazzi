#include <glib.h>

#include "booz_flight_model.h"

/* 250Hz <-> 4ms */
#define TIMEOUT_PERIOD 4
#define DT (TIMEOUT_PERIOD*1e-3)
double sim_time;

double commands[] = {0.7, 0.7, 0.7, 0.7};

gboolean timeout_callback(gpointer data) {

  booz_flight_model_run(DT, commands);
  sim_time += DT;

  printf("%f %f %f %f %f\n", sim_time, bs.mot_omega->ve[0], bs.mot_omega->ve[1],
	 bs.mot_omega->ve[2], bs.mot_omega->ve[3]);

  return TRUE;
}

int main ( int argc, char** argv) {

  sim_time = 0.;
  booz_flight_model_init();


  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(TIMEOUT_PERIOD, timeout_callback, NULL);
  
  g_main_loop_run(ml);
 
  return 0;
}
