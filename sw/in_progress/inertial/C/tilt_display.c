#include "tilt_display.h"

#include <gtkdatabox.h>
#include <gtkdatabox_points.h>
#include <gtkdatabox_lines.h>
#include <gtkdatabox_bars.h>
#include <gtkdatabox_grid.h>
#include <gtkdatabox_cross_simple.h>
#include <gtkdatabox_marker.h>

GtkWidget* tilt_display(struct tilt_data* td) {
  GtkWidget* window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_widget_set_size_request (window, 1280, 800);

  GtkWidget* vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), vbox1);

  GtkWidget *frame = gtk_frame_new ("rate");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);

  GtkWidget* databox = gtk_databox_new();
  gfloat* prate = g_new0 (gfloat, td->nb_samples);
  gfloat* p_m_rate = g_new0 (gfloat, td->nb_samples);
  gfloat* pt = g_new0 (gfloat, td->nb_samples);
  int idx;
  for (idx=0; idx<td->nb_samples; idx++) {
    prate[idx] = td->rate[idx];
    p_m_rate[idx] = td->gyro[idx];
    pt[idx] = td->t[idx];
  }
  GdkColor red = {0, 65535, 0, 0};
  GdkColor green = {0, 0, 65535, 0};
  GdkColor blue = {0, 0, 0, 65535};
  GdkColor pink = {0, 65535, 0, 65535};
  GdkColor black = {0, 0, 0, 0};

  GtkDataboxGraph *grate = gtk_databox_lines_new (td->nb_samples, pt, prate, &red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), grate);

  GtkDataboxGraph *g_m_rate = gtk_databox_lines_new (td->nb_samples, pt, p_m_rate, &green, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_m_rate);

  GtkDataboxGraph *grid = gtk_databox_grid_new (10, 10, &black, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), grid);

  gtk_container_add (GTK_CONTAINER (frame), databox );
  gtk_databox_auto_rescale (GTK_DATABOX(databox), 0.);



  frame = gtk_frame_new ("angle");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);

  databox = gtk_databox_new();
  gfloat* pangle = g_new0 (gfloat, td->nb_samples);
  gfloat* pm_angle = g_new0 (gfloat, td->nb_samples);
  gfloat* pe_angle = g_new0 (gfloat, td->nb_samples);
  for (idx=0; idx<td->nb_samples; idx++) {
    pangle[idx] = td->angle[idx];
    pm_angle[idx] = td->m_angle[idx];
    pe_angle[idx] = td->est_angle[idx];
  }

  GtkDataboxGraph *gangle = gtk_databox_lines_new (td->nb_samples, pt, pangle, &red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), gangle);

  GtkDataboxGraph *ge_angle = gtk_databox_lines_new (td->nb_samples, pt, pe_angle, &blue, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), ge_angle);

  GtkDataboxGraph *gm_angle = gtk_databox_lines_new (td->nb_samples, pt, pm_angle, &green, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), gm_angle);

  grid = gtk_databox_grid_new (10, 10, &black, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), grid);

  gtk_container_add (GTK_CONTAINER (frame), databox );
  gtk_databox_auto_rescale (GTK_DATABOX(databox), 0.);


  frame = gtk_frame_new ("bias");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);

  databox = gtk_databox_new();
  gfloat* pbias = g_new0 (gfloat, td->nb_samples);
  gfloat* pest_bias = g_new0 (gfloat, td->nb_samples);
  for (idx=0; idx<td->nb_samples; idx++) {
    pbias[idx] = td->bias[idx];
    pest_bias[idx] = td->est_bias[idx];
  }
  GtkDataboxGraph *gest_bias = gtk_databox_lines_new (td->nb_samples, pt, pest_bias, &blue, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), gest_bias);

  GtkDataboxGraph *gbias = gtk_databox_lines_new (td->nb_samples, pt, pbias, &red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), gbias);

  grid = gtk_databox_grid_new (10, 10, &black, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), grid);

  gtk_container_add (GTK_CONTAINER (frame), databox );
  gtk_databox_auto_rescale (GTK_DATABOX(databox), 0.);


  frame = gtk_frame_new ("covariance");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);

  databox = gtk_databox_new();
  gfloat* pP00 = g_new0 (gfloat, td->nb_samples);
  gfloat* pP01 = g_new0 (gfloat, td->nb_samples);
  gfloat* pP10 = g_new0 (gfloat, td->nb_samples);
  gfloat* pP11 = g_new0 (gfloat, td->nb_samples);
  for (idx=0; idx<td->nb_samples; idx++) {
    pP00[idx] = td->P00[idx];
    pP01[idx] = td->P01[idx];
    pP10[idx] = td->P10[idx];
    pP11[idx] = td->P11[idx];
  }
  GtkDataboxGraph *gP00 = gtk_databox_lines_new (td->nb_samples, pt, pP00, &red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), gP00);

  GtkDataboxGraph *gP01 = gtk_databox_lines_new (td->nb_samples, pt, pP01, &green, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), gP01);

  GtkDataboxGraph *gP10 = gtk_databox_lines_new (td->nb_samples, pt, pP10, &blue, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), gP10);

  GtkDataboxGraph *gP11 = gtk_databox_lines_new (td->nb_samples, pt, pP11, &pink, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), gP11);

  grid = gtk_databox_grid_new (10, 10, &black, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), grid);

  gtk_container_add (GTK_CONTAINER (frame), databox );
  gtk_databox_auto_rescale (GTK_DATABOX(databox), 0.);


  gtk_widget_show_all(window);
  return window;
}

