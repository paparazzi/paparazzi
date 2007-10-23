#include <glib.h>

#include "booz_flight_model.h"
#include "booz_flightgear.h"


/* 250Hz <-> 4ms */
#define TIMEOUT_PERIOD 4
#define DT (TIMEOUT_PERIOD*1e-3)
double sim_time;

#define DT_DISPLAY 0.04
double next_display;


double commands[] = {0.651, 0.649, 0.65, 0.65};

static void output_task( void );
static gboolean timeout_callback(gpointer data);

static gboolean timeout_callback(gpointer data) {

  booz_flight_model_run(DT, commands);
  sim_time += DT;

  if (sim_time >= next_display) {
    next_display+= DT_DISPLAY;
    //    output_task();
    booz_flightgear_send();
  }

  return TRUE;
}

static void output_task( void ) {
  printf("%f %f %f %f %f %f %f\n", sim_time, 
	 bfm.state->ve[BFMS_X], bfm.state->ve[BFMS_Y], bfm.state->ve[BFMS_Z], 
	 bfm.state->ve[BFMS_PHI], bfm.state->ve[BFMS_THETA], bfm.state->ve[BFMS_PSI]);


}


int main ( int argc, char** argv) {

  sim_time = 0.;
  next_display = 0.;
  booz_flight_model_init();

  booz_flightgear_init("10.31.4.107", 5501);


  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  g_timeout_add(TIMEOUT_PERIOD, timeout_callback, NULL);
  
  g_main_loop_run(ml);
 
  return 0;
}
