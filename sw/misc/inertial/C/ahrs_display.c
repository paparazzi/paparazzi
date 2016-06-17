#include "ahrs_display.h"

#include <gtkdatabox.h>
#include <gtkdatabox_points.h>
#include <gtkdatabox_lines.h>
#include <gtkdatabox_bars.h>
#include <gtkdatabox_grid.h>
#include <gtkdatabox_cross_simple.h>
#include <gtkdatabox_marker.h>

GtkWidget* ahrs_display(struct ahrs_data* ad) {
  GtkWidget* window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_widget_set_size_request (window, 1280, 800);

  GtkWidget* vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), vbox1);

  GdkColor red = {0, 65535, 0, 0};
  GdkColor light_red = {0, 65535, 31875, 31875};
  GdkColor green = {0, 0, 65535, 0};
  GdkColor light_green = {0, 31875, 65535, 31875};
  GdkColor blue = {0, 0, 0, 65535};
  GdkColor light_blue = {0, 14025, 39525, 65535};
  //  GdkColor pink = {0, 65535, 0, 65535};
  //  GdkColor black = {0, 0, 0, 0};

  GtkWidget *frame = gtk_frame_new ("rate");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);

  GtkWidget* databox = gtk_databox_new();
  gfloat* ft = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_mrate_p = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_mrate_q = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_mrate_r = g_new0 (gfloat, ad->nb_samples);
  int idx;
  for (idx=0; idx<ad->nb_samples; idx++) {
    ft[idx] = ad->t[idx];
    f_mrate_p[idx] = ad->gyro_p[idx];
    f_mrate_q[idx] = ad->gyro_q[idx];
    f_mrate_r[idx] = ad->gyro_r[idx];
  }
  GtkDataboxGraph *g_mrate_p = gtk_databox_lines_new (ad->nb_samples, ft, f_mrate_p, &red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_mrate_p);
  GtkDataboxGraph *g_mrate_q = gtk_databox_lines_new (ad->nb_samples, ft, f_mrate_q, &green, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_mrate_q);
  GtkDataboxGraph *g_mrate_r = gtk_databox_lines_new (ad->nb_samples, ft, f_mrate_r, &blue, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_mrate_r);

  gtk_container_add (GTK_CONTAINER (frame), databox );
  gtk_databox_auto_rescale (GTK_DATABOX(databox), 0.);



  frame = gtk_frame_new ("angle");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);

  databox = gtk_databox_new();
  //  databox = gtk_databox_create_box_with_scrollbars_and_rulers();
  gfloat* f_ephi = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_etheta = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_epsi = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_mphi = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_mtheta = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_mpsi = g_new0 (gfloat, ad->nb_samples);
  for (idx=0; idx<ad->nb_samples; idx++) {
    f_ephi[idx] = ad->est_phi[idx];
    f_etheta[idx] = ad->est_theta[idx];
    f_epsi[idx] = ad->est_psi[idx];
    f_mphi[idx] = ad->m_phi[idx];
    f_mtheta[idx] = ad->m_theta[idx];
    f_mpsi[idx] = ad->m_psi[idx];
  }
  GtkDataboxGraph *g_ephi = gtk_databox_lines_new (ad->nb_samples, ft, f_ephi, &red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_ephi);
  GtkDataboxGraph *g_etheta = gtk_databox_lines_new (ad->nb_samples, ft, f_etheta, &green, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_etheta);
  GtkDataboxGraph *g_epsi = gtk_databox_lines_new (ad->nb_samples, ft, f_epsi, &blue, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_epsi);
  GtkDataboxGraph *g_mphi = gtk_databox_lines_new (ad->nb_samples, ft, f_mphi, &light_red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_mphi);
  GtkDataboxGraph *g_mtheta = gtk_databox_lines_new (ad->nb_samples, ft, f_mtheta, &light_green, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_mtheta);
  GtkDataboxGraph *g_mpsi = gtk_databox_lines_new (ad->nb_samples, ft, f_mpsi, &light_blue, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_mpsi);

  gtk_container_add (GTK_CONTAINER (frame), databox );
  gtk_databox_auto_rescale (GTK_DATABOX(databox), 0.);



  frame = gtk_frame_new ("biase");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);

  databox = gtk_databox_new();
  gfloat* f_ebias_p = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_ebias_q = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_ebias_r = g_new0 (gfloat, ad->nb_samples);
  for (idx=0; idx<ad->nb_samples; idx++) {
    f_ebias_p[idx] = ad->est_bias_p[idx];
    f_ebias_q[idx] = ad->est_bias_q[idx];
    f_ebias_r[idx] = ad->est_bias_r[idx];
  }
  GtkDataboxGraph *ge_bias_p = gtk_databox_lines_new (ad->nb_samples, ft, f_ebias_p, &red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), ge_bias_p);
  GtkDataboxGraph *ge_bias_q = gtk_databox_lines_new (ad->nb_samples, ft, f_ebias_q, &green, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), ge_bias_q);
  GtkDataboxGraph *ge_bias_r = gtk_databox_lines_new (ad->nb_samples, ft, f_ebias_r, &blue, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), ge_bias_r);

  gtk_container_add (GTK_CONTAINER (frame), databox );
  gtk_databox_auto_rescale (GTK_DATABOX(databox), 0.);



  frame = gtk_frame_new ("covariance");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);

  databox = gtk_databox_new();
  gfloat* f_p11 = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_p22 = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_p33 = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_p44 = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_p55 = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_p66 = g_new0 (gfloat, ad->nb_samples);
  gfloat* f_p77 = g_new0 (gfloat, ad->nb_samples);
  for (idx=0; idx<ad->nb_samples; idx++) {
    f_p11[idx] = ad->P11[idx];
    f_p22[idx] = ad->P22[idx];
    f_p33[idx] = ad->P33[idx];
    f_p44[idx] = ad->P44[idx];
    f_p55[idx] = ad->P55[idx];
    f_p66[idx] = ad->P66[idx];
    f_p77[idx] = ad->P77[idx];
  }
  GtkDataboxGraph *g_p11 = gtk_databox_lines_new (ad->nb_samples, ft, f_p11, &red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_p11);
  GtkDataboxGraph *g_p22 = gtk_databox_lines_new (ad->nb_samples, ft, f_p22, &red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_p22);
  GtkDataboxGraph *g_p33 = gtk_databox_lines_new (ad->nb_samples, ft, f_p33, &red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_p33);
  GtkDataboxGraph *g_p44 = gtk_databox_lines_new (ad->nb_samples, ft, f_p44, &red, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_p44);
  GtkDataboxGraph *g_p55 = gtk_databox_lines_new (ad->nb_samples, ft, f_p55, &blue, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_p55);
  GtkDataboxGraph *g_p66 = gtk_databox_lines_new (ad->nb_samples, ft, f_p66, &blue, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_p66);
  GtkDataboxGraph *g_p77 = gtk_databox_lines_new (ad->nb_samples, ft, f_p77, &blue, 1);
  gtk_databox_graph_add (GTK_DATABOX(databox), g_p77);

  gtk_container_add (GTK_CONTAINER (frame), databox );
  gtk_databox_auto_rescale (GTK_DATABOX(databox), 0.);


  gtk_widget_show_all(window);
  return window;
}
