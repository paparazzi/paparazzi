#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <gtkdatabox.h>
#include <gtkdatabox_points.h>
#include <gtkdatabox_lines.h>
#include <gtkdatabox_bars.h>
#include <gtkdatabox_grid.h>
#include <gtkdatabox_cross_simple.h>
#include <gtkdatabox_marker.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define NB_POINTS 2000

GtkWidget* databox;
static gfloat *X;
static gfloat *BX;
static gfloat *BY;
static gfloat *BZ;
static guint nb_data;

GtkWidget* plot_frame_new(void) {
  databox = gtk_databox_new ();

  X = g_new0 (gfloat, NB_POINTS);
  BX = g_new0 (gfloat, NB_POINTS);
  BY = g_new0 (gfloat, NB_POINTS);
  BZ = g_new0 (gfloat, NB_POINTS);

  GdkColor color;
  color.red = 0;
  color.green = 0;
  color.blue = 0;
  GtkDataboxGraph* grid = gtk_databox_grid_new (7, 7, &color, 2);
  gtk_databox_graph_add (GTK_DATABOX (databox), grid);

  color.red = 0;
  color.green = 65535;
  color.blue = 0;
  GtkDataboxGraph *graph_bx = gtk_databox_lines_new (NB_POINTS, X, BX, &color, 1);
  gtk_databox_graph_add (GTK_DATABOX (databox), graph_bx);

  color.red = 65535;
  color.green = 0;
  color.blue = 0;
  GtkDataboxGraph *graph_by = gtk_databox_lines_new(NB_POINTS, X, BY, &color, 1);
  gtk_databox_graph_add (GTK_DATABOX (databox), graph_by);

  color.red = 0;
  color.green = 0;
  color.blue = 65535;
  GtkDataboxGraph *graph_bz = gtk_databox_lines_new(NB_POINTS, X, BZ, &color, 1);
  gtk_databox_graph_add (GTK_DATABOX (databox), graph_bz);

  nb_data = 0;

  return databox;

}

void update_state_plot(float q0, float q1, float q2, float q3, float bx, float by, float bz) {
  if (nb_data < NB_POINTS) {
    X[nb_data] = nb_data;
    BX[nb_data] = bx;
    BY[nb_data] = by;
    BZ[nb_data] = bz;
    gtk_databox_auto_rescale (GTK_DATABOX (databox), 0.);
    nb_data++;
  }
}


void on_AHRS_STATE(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  float q0 = atof(argv[0]);
  float q1 = atof(argv[1]);
  float q2 = atof(argv[2]);
  float q3 = atof(argv[3]);
  float bx = atof(argv[4]);
  float by = atof(argv[5]);
  float bz = atof(argv[6]);
  update_state_plot(q0, q1, q2, q3, bx, by, bz);


}

int main (int argc, char** argv) {

  gtk_init(&argc, &argv); 
  
  GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_widget_set_size_request (window, 1240, 600);

  GtkWidget *vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), vbox1);
  gtk_box_pack_start (GTK_BOX (vbox1), plot_frame_new(), TRUE, TRUE, 0);

  gtk_widget_show_all(window);


  IvyInit ("IvyGtkButton", "IvyGtkButton READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_AHRS_STATE, NULL, "^77 AHRS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  //IvyBindMsg(on_AHRS_STATE, NULL, "^77 AHRS (.*)");
  IvyStart("127.255.255.255");

  gtk_main();
  return 0;
}
