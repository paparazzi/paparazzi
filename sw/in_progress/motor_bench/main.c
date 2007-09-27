#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

//#include "sliding_plot.h"


GtkWidget* build_gui ( void ) {

  GtkWidget *window1;
  GtkWidget *vbox1;


  window1 = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window1), "motor_bench");

  vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window1), vbox1);

  return window1;
}

#if 0
void on_scale_value_changed (GtkScale  *scale, gpointer user_data) {
  gfloat cf = gtk_range_get_value(GTK_RANGE(scale));
  gint c = round(cf);
  g_message("foo %d %f", user_data, c);
  IvySendMsg("ME RAW_DATALINK 16 SETTING;%d;0;%d", (gint)user_data, c);
}
#endif


void on_MOTOR_BENCH_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  int time_ticks =  atoi(argv[0]);
  g_message("foo %d", time_ticks);

}

int main (int argc, char** argv) {

  gtk_init(&argc, &argv);

  GtkWidget* window = build_gui();
  gtk_widget_show_all(window);

  IvyInit ("MotorBench", "MotorBench READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_MOTOR_BENCH_STATUS, NULL, "^\\S* MOTOR_BENCH_STATUS (\\S*) (\\S*) (\\S*) (\\S*),(\\S*),(\\S*)");
  IvyStart("127.255.255.255");

  gtk_main();
  return 0;
}
