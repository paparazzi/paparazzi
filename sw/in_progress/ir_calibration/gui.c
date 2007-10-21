#include "gui.h"

#include <gtkdatabox.h>
#include <gtkdatabox_points.h>
#include <gtkdatabox_lines.h>
#include <gtkdatabox_bars.h>
#include <gtkdatabox_grid.h>
#include <gtkdatabox_cross_simple.h>
#include <gtkdatabox_marker.h>

#define NB_POINTS 121

gfloat *X;
gfloat *Y1;
gfloat *Y2;

static GtkWidget* databox;

GtkWidget* gui_init(void) {

  GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_widget_set_size_request (window, 640, 400);

  GtkWidget *vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), vbox1);
  
  GtkWidget *frame = gtk_frame_new ("Foo");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);
  
  databox = gtk_databox_new ();
  gtk_container_add (GTK_CONTAINER (frame), databox );

  X = g_new0 (gfloat, NB_POINTS);
  Y1 = g_new0 (gfloat, NB_POINTS);
  Y2 = g_new0 (gfloat, NB_POINTS);
  guint i;
  for (i=0; i<NB_POINTS; i++){  
    X[i] = (gfloat)i - NB_POINTS/2; 
    Y1[i] = (gfloat)i -NB_POINTS/2;
    Y2[i] = (gfloat)i -NB_POINTS/2;
  }

  GdkColor cblack = {0, 0, 0, 0};
  GdkColor cred =   {0, 65535, 0, 0};

  GtkDataboxGraph *graph1 = gtk_databox_lines_new (NB_POINTS, X, Y1, &cblack, 2);
  gtk_databox_graph_add (GTK_DATABOX (databox), graph1);

  GtkDataboxGraph *graph2 = gtk_databox_lines_new (NB_POINTS, X, Y2, &cred, 2);
  gtk_databox_graph_add (GTK_DATABOX (databox), graph2);

  GtkDataboxGraph *grid = gtk_databox_grid_new (11, 11, &cblack, 1);
  gtk_databox_graph_add (GTK_DATABOX (databox), grid); 

  gtk_databox_auto_rescale (GTK_DATABOX(databox), 0.);

  gtk_widget_show_all(window);
  return window;
}

void gui_update(gfloat* values) {
  guint i;
  for (i=0; i<NB_POINTS; i++){  
    Y2[i] = values[i];
  }
  gtk_databox_auto_rescale (GTK_DATABOX(databox), 0.);
}
