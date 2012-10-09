#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include <math.h>

gint ac_id = 42;
GtkWidget *spin; /* awfull but a lot easier */

void on_scale_value_changed (GtkScale  *scale, gpointer user_data) {
  gfloat cf = gtk_range_get_value(GTK_RANGE(scale));
  gint c = (gint)rint(cf);

  gfloat sf = gtk_spin_button_get_value (( GtkSpinButton*)spin);
  gint s = (gint)rint(sf);

  IvySendMsg("actuators_gtk RAW_DATALINK %d SET_ACTUATOR;%d;%d;%d", ac_id, c, s, ac_id);
}

GtkWidget* build_gui ( void ) {
  GtkWidget *window1;
  GtkWidget *vbox1;

  window1 = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window1), "Set Actuators");
  gtk_window_set_default_size(GTK_WINDOW (window1), 320, -1);

  vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window1), vbox1);

  GtkWidget *scale = gtk_hscale_new (GTK_ADJUSTMENT (gtk_adjustment_new (1500, 1000, 2000, 1, 1, 0)));
  gtk_box_pack_start (GTK_BOX (vbox1), scale, TRUE, TRUE, 0);
  gtk_range_set_update_policy (GTK_RANGE (scale), GTK_UPDATE_DELAYED);
  g_signal_connect ((gpointer) scale, "value_changed",
                    G_CALLBACK (on_scale_value_changed),
                    (gpointer)0);

  spin = gtk_spin_button_new(GTK_ADJUSTMENT (gtk_adjustment_new (0, 0, 8, 1, 1, 0)), 1, 0);
  gtk_box_pack_start (GTK_BOX (vbox1), spin, TRUE, TRUE, 0);

  return window1;
}


int main (int argc, char** argv) {
  gtk_init(&argc, &argv);

  if (argc > 1) {
    ac_id = atoi(argv[1]);
  } else {
    g_message("Warning: A/C id not specified; sending to %d", ac_id);
  }

  GtkWidget* window = build_gui();
  gtk_widget_show_all(window);

  IvyInit ("Setup Actuators", "Actuators READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  gtk_main();
  return 0;
}
