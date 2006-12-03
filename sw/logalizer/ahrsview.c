#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "sliding_plot.h"

GtkWidget *biases_plot, *eulers_plot;

void quat_to_euler( gfloat* quat, gfloat* euler) {
  //  float q02 = quat[0] * quat[0];
  float q12 = quat[1] * quat[1];
  float q22 = quat[2] * quat[2];
  float q32 = quat[3] * quat[3];

  euler[0] = atan2( 2*(quat[2]*quat[3] + quat[0]*quat[1]),(1-2*(q12 + q22)) );
  euler[1] = -asin(2*(quat[1]*quat[3] - quat[0]*quat[2]));
  euler[2] = atan2( 2*(quat[1]*quat[2] + quat[0]*quat[3]),(1-2*(q22 + q32)) );
}

void on_AHRS_STATE(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  float q0 = atof(argv[0]);
  float q1 = atof(argv[1]);
  float q2 = atof(argv[2]);
  float q3 = atof(argv[3]);
  float bx = atof(argv[4]);
  float by = atof(argv[5]);
  float bz = atof(argv[6]);
  gfloat biases[] = {bx, by, bz};
  gfloat quat[] = {q0, q1, q2, q3};
  gfloat eulers[] = {0., 0., 0.};
  quat_to_euler(quat, eulers);
  //  sliding_plot_update(GTK_SLIDING_PLOT(biases_plot), foo1);
  //  sliding_plot_update(GTK_SLIDING_PLOT(quat_plot), foo2);
  sliding_plot_update(biases_plot, biases);
  sliding_plot_update(eulers_plot, eulers);
}

int main (int argc, char** argv) {

  gtk_init(&argc, &argv); 
  
  GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_widget_set_size_request (window, 1280, 480);

  GtkWidget *vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), vbox1);
 
  GtkWidget *frame = gtk_frame_new ("Biases");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);
  biases_plot = sliding_plot_new(3);
  gtk_container_add (GTK_CONTAINER (frame), biases_plot );

  frame = gtk_frame_new ("Eulers");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);
  eulers_plot = sliding_plot_new(3);
  gtk_container_add (GTK_CONTAINER (frame), eulers_plot );

  gtk_widget_show_all(window);


  IvyInit ("IvyGtkButton", "IvyGtkButton READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_AHRS_STATE, biases_plot, "^77 AHRS_STATE (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  gtk_main();
  return 0;
}
