#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "sliding_plot.h"


GtkWidget *scale1, *scale2, *scale3;
GtkWidget *lab1, *lab2, *lab3;
GtkWidget *plot;

void on_scale_value_changed (GtkScale  *scale, gpointer user_data) {
  gfloat cf = gtk_range_get_value(GTK_RANGE(scale));
  gint c = round(cf);
  g_message("foo %d %f", user_data, c);
  IvySendMsg("ME RAW_DATALINK 16 SETTING;%d;0;%d", (gint)user_data, c);
}

GtkWidget* build_gui ( void ) {
  GtkWidget *window1;
  GtkWidget *vbox1;
  GtkWidget *table1;

  window1 = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window1), "tracking antenna");

  vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window1), vbox1);

  table1 = gtk_table_new (2, 3, FALSE);
  gtk_box_pack_start (GTK_BOX (vbox1), table1, TRUE, TRUE, 0);
  gtk_table_set_col_spacings (GTK_TABLE (table1), 5);

  lab1 = gtk_label_new ("XXXX");
  gtk_table_attach (GTK_TABLE (table1), lab1, 0, 1, 0, 1,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (lab1), 0, 0.5);

  lab2 = gtk_label_new ("XXXX");
  gtk_table_attach (GTK_TABLE (table1), lab2, 0, 1, 1, 2,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (lab2), 0, 0.5);


  lab3 = gtk_label_new ("XXXX");
  gtk_table_attach (GTK_TABLE (table1), lab3, 0, 1, 2, 3,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (lab3), 0, 0.5);



  scale1 = gtk_hscale_new (GTK_ADJUSTMENT (gtk_adjustment_new (0, 0, 255, 1, 1, 1)));
  gtk_table_attach (GTK_TABLE (table1), scale1, 1, 2, 0, 1,
                    (GtkAttachOptions) (GTK_EXPAND | GTK_FILL),
                    (GtkAttachOptions) (GTK_FILL), 0, 0);
  gtk_range_set_update_policy (GTK_RANGE (scale1), GTK_UPDATE_DELAYED);
  g_signal_connect ((gpointer) scale1, "value_changed",
                    G_CALLBACK (on_scale_value_changed),
                    (gpointer)0);


  scale2 = gtk_hscale_new (GTK_ADJUSTMENT (gtk_adjustment_new (0, 0, 255, 1, 1, 1)));
  gtk_table_attach (GTK_TABLE (table1), scale2, 1, 2, 1, 2,
                    (GtkAttachOptions) (GTK_EXPAND | GTK_FILL),
                    (GtkAttachOptions) (GTK_FILL), 0, 0);
  gtk_range_set_update_policy (GTK_RANGE (scale2), GTK_UPDATE_DELAYED);
  g_signal_connect ((gpointer) scale2, "value_changed",
                    G_CALLBACK (on_scale_value_changed),
                    (gpointer)1);

  scale3 = gtk_hscale_new (GTK_ADJUSTMENT (gtk_adjustment_new (0, 0, 255, 1, 1, 1)));
  gtk_table_attach (GTK_TABLE (table1), scale3, 1, 2, 2, 3,
                    (GtkAttachOptions) (GTK_EXPAND | GTK_FILL),
                    (GtkAttachOptions) (GTK_FILL), 0, 0);
  gtk_range_set_update_policy (GTK_RANGE (scale3), GTK_UPDATE_DELAYED);
  g_signal_connect ((gpointer) scale3, "value_changed",
                    G_CALLBACK (on_scale_value_changed),
                    (gpointer)2);


  GtkWidget *frame = gtk_frame_new ("SaMere");
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);
  plot = sliding_plot_new(3);
  gtk_container_add (GTK_CONTAINER (frame), plot );

  return window1;


}

void on_ENOSE_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  int v1 =  atoi(argv[0]);
  int v2 =  atoi(argv[1]);
  int v3 =  atoi(argv[2]);
  gfloat foo[] = {(gfloat)v1, (gfloat)v2, (gfloat)v3};
  sliding_plot_update(plot, foo);

  int c1 =  atoi(argv[3]);
  int c2 =  atoi(argv[4]);
  int c3 =  atoi(argv[5]);
  
  GString* str= g_string_sized_new(64);
  g_string_printf(str, "%d", c1);
  gtk_label_set_text(GTK_LABEL(lab1), str->str);
  g_string_printf(str, "%d", c2);
  gtk_label_set_text(GTK_LABEL(lab2), str->str);
  g_string_printf(str, "%d", c3);
  gtk_label_set_text(GTK_LABEL(lab3), str->str);
  g_string_free(str, TRUE);

}

int main (int argc, char** argv) {

  gtk_init(&argc, &argv);

  GtkWidget* window = build_gui();
  gtk_widget_show_all(window);

  IvyInit ("TestEnose", "TestEnose READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_ENOSE_STATUS, NULL, "^\\S* ENOSE_STATUS (\\S*) (\\S*) (\\S*) (\\S*),(\\S*),(\\S*)");
  IvyStart("127.255.255.255");

  gtk_main();
  return 0;
}
