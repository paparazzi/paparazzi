#include <gtkdatabox.h>
#include <gtkdatabox_points.h>
#include <gtkdatabox_lines.h>
#include <gtkdatabox_bars.h>
#include <gtkdatabox_grid.h>
#include <gtkdatabox_cross_simple.h>
#include <gtkdatabox_markers.h>

#include <string.h>

#define NB_POINTS 2000
#define NB_POINTS_BY_STEP 1
//500

const GdkColor colors[] = {{65535, 0, 0}, {0, 65535, 0}, {0, 0, 65535}, {0, 0, 0}};
#define NB_COLORS 4

#define REFRESH_RATE 166   /* ms */
static gboolean timeout_callback(gpointer data);

GtkWidget* sliding_plot_new(guint nb_plot) {
  GtkWidget* databox = gtk_databox_new ();
  g_object_set_data(G_OBJECT(databox), "nb_plot", (gpointer)nb_plot);
  gfloat *X = g_new0 (gfloat, NB_POINTS);
  g_object_set_data(G_OBJECT(databox), "X", X);
  gpointer *Y = g_new0 (gpointer, nb_plot);
  g_object_set_data(G_OBJECT(databox), "Y", Y);
  guint nb_data = 0;
  g_object_set_data(G_OBJECT(databox), "nb_data", (gpointer)nb_data);
  guint _time = 0;
  g_object_set_data(G_OBJECT(databox), "time", (gpointer)_time);
  guint i;
  for (i=0; i< nb_plot; i++) {
    Y[i] = g_new0 (gfloat, NB_POINTS);
    GtkDataboxGraph *graph = gtk_databox_lines_new (NB_POINTS, X, Y[i], (GdkColor*)&(colors[i%NB_COLORS]), 1);
    gtk_databox_graph_add (GTK_DATABOX (databox), graph);
  }

  //  GtkDataboxGraph *grid = gtk_databox_grid_new (10, 10, (GdkColor*)&(colors[3]), 2);
  //  gtk_databox_graph_add (GTK_DATABOX (databox), grid);
    g_timeout_add(REFRESH_RATE, timeout_callback, databox);

  return databox;
}

void sliding_plot_update(GtkWidget* plot, float* values) {
  guint i, j;
  gfloat *X = g_object_get_data(G_OBJECT(plot), "X");
  gpointer *Y = g_object_get_data(G_OBJECT(plot), "Y");
  guint nb_data = (guint)g_object_get_data(G_OBJECT(plot), "nb_data");
  guint _time = (guint)g_object_get_data(G_OBJECT(plot), "time");
  guint nb_plot = (guint)g_object_get_data(G_OBJECT(plot), "nb_plot");

  if (nb_data >= NB_POINTS) {
    nb_data -= NB_POINTS_BY_STEP;
    guint mem_to_move = (NB_POINTS - NB_POINTS_BY_STEP)*sizeof(gfloat);
    memmove(X, &(X[NB_POINTS_BY_STEP]), mem_to_move);
    for (i=0; i< nb_plot; i++) {
      gfloat* y = Y[i];
      memmove(y, &(y[NB_POINTS_BY_STEP]), mem_to_move);
    }
  }

  for (j=nb_data; j< NB_POINTS; j++) {
    X[j] = _time;
    for (i=0; i< nb_plot; i++) {
      gfloat* y = Y[i];
      y[j] = values[i];
    }
  }
  _time++;
  g_object_set_data(G_OBJECT(plot), "time", (gpointer)_time);
  nb_data++;
  g_object_set_data(G_OBJECT(plot), "nb_data", (gpointer)nb_data);
  //  gtk_databox_auto_rescale (GTK_DATABOX (plot), 0.);
}


static gboolean timeout_callback(gpointer user_data) {
  GtkDatabox* databox = GTK_DATABOX (user_data);
  gtk_databox_auto_rescale (databox, 0.);
  return TRUE;
}
