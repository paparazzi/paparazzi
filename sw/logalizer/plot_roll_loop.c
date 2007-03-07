#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "sliding_plot.h"


GtkWidget *plot;

GtkWidget* build_gui ( void ) {
  GtkWidget *window1;
  GtkWidget *vbox1;

  window1 = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window1), "plot roll loop");
  
  vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window1), vbox1);

  GtkWidget *frame = gtk_frame_new ("SaSeur");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);
  plot = sliding_plot_new(2);
  gtk_container_add (GTK_CONTAINER (frame), plot );

  return window1;


}


void on_DESIRED(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  //   float p =  atof(argv[0]);
  float phi =  atof(argv[1]);
  float phi_sp =  atof(argv[2]);

  gfloat foo[] = {(gfloat)phi, (gfloat)phi_sp};
  sliding_plot_update(plot, foo);

}

int main (int argc, char** argv) {

  gtk_init(&argc, &argv);

  GtkWidget* window = build_gui();
  gtk_widget_show_all(window);

  IvyInit ("PlotRollLoop", "PlotRollLoop READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_DESIRED, NULL, "^\\S* TUNE_ROLL (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  gtk_main();
  return 0;
}
