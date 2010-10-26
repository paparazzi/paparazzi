#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "sliding_plot.h"

GtkWidget *mag_plot;
GtkWidget *gyro_plot;
GtkWidget *accel_plot;

void on_IMU_MAG(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  float mx = atof(argv[0]);
  float my = atof(argv[1]);
  float mz = atof(argv[2]);
  gfloat mag[] = {mx, my, mz};
  sliding_plot_update(mag_plot, mag);
}

void on_IMU_ACCEL(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  float ax = atof(argv[0]);
  float ay = atof(argv[1]);
  float az = atof(argv[2]);
  gfloat accel[] = {ax, ay, az};
  sliding_plot_update(accel_plot, accel);
}

void on_IMU_GYRO(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  float gx = atof(argv[0]);
  float gy = atof(argv[1]);
  float gz = atof(argv[2]);
  gfloat rates[] = {gx, gy, gz};
  sliding_plot_update(gyro_plot, rates);
}

int main (int argc, char** argv) {

  gtk_init(&argc, &argv); 
  
  GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_widget_set_size_request (window, 1280, 480);

  GtkWidget *vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), vbox1);
 
  GtkWidget *frame = gtk_frame_new ("Mag");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);
  mag_plot = sliding_plot_new(3);
  gtk_container_add (GTK_CONTAINER (frame), mag_plot );

  frame = gtk_frame_new ("Accel");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);
  accel_plot = sliding_plot_new(3);
  gtk_container_add (GTK_CONTAINER (frame), accel_plot );

  frame = gtk_frame_new ("Gyro");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);
  gyro_plot = sliding_plot_new(3);
  gtk_container_add (GTK_CONTAINER (frame), gyro_plot );

  gtk_widget_show_all(window);


  IvyInit ("imuview", "imuview READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_IMU_MAG, NULL, "^77 IMU_MAG (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_IMU_ACCEL, NULL, "^77 IMU_ACCEL (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_IMU_GYRO, NULL, "^77 IMU_GYRO (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  gtk_main();
  return 0;
}
