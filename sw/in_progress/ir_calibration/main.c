#include <glib.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


#include "calibrator.h"
#include "gui.h"

gboolean timeout_callback(gpointer data) {

  float* values = calibrator_get_values();
  gui_update(values);
  return TRUE;

}


int main ( int argc, char** argv) {
  
  //  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  gtk_init(&argc, &argv);

  g_timeout_add(500, timeout_callback, NULL);

  calibrator_init();
  
  gui_init();

  //g_main_loop_run(ml);
  gtk_main();

  return 0;
}
