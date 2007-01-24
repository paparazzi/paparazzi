#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define MANUAL 0
#define AUTO 1

static float azim_sp;
static float elev_sp;
static guint id_sp;
static guint mode;

GtkWidget *azim_scale;
GtkWidget *elev_scale;

void on_mode_changed (GtkRadioButton  *radiobutton, gpointer user_data) {

  mode = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radiobutton)) ? MANUAL : AUTO;
  IvySendMsg("1ME RAW_DATALINK 80 SETTING;0;0;%d", mode);
  g_message("on mode changed %d" , mode);  
}


#define GLADE_HOOKUP_OBJECT(component,widget,name) \
  g_object_set_data_full (G_OBJECT (component), name, \
    gtk_widget_ref (widget), (GDestroyNotify) gtk_widget_unref)

#define GLADE_HOOKUP_OBJECT_NO_REF(component,widget,name) \
  g_object_set_data (G_OBJECT (component), name, widget)


GtkWidget* build_gui ( void ) {
  GtkWidget *window1;
  GtkWidget *vbox1;
  GtkWidget *vbox2;
  GtkWidget *table1;
  GtkWidget *label1;
  GtkWidget *label2;
  GtkWidget *label3;
  GtkWidget *label4;
  GtkWidget *radiobutton1;
  GSList *radiobutton1_group = NULL;
  GtkWidget *radiobutton2;
  GtkWidget *entry1;

  window1 = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window1), "tracking antenna");

  vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_widget_show (vbox1);
  gtk_container_add (GTK_CONTAINER (window1), vbox1);

  vbox2 = gtk_vbox_new (FALSE, 0);
  gtk_widget_show (vbox2);
  gtk_box_pack_start (GTK_BOX (vbox1), vbox2, TRUE, TRUE, 0);

  table1 = gtk_table_new (4, 3, FALSE);
  gtk_widget_show (table1);
  gtk_box_pack_start (GTK_BOX (vbox2), table1, TRUE, TRUE, 0);
  gtk_table_set_col_spacings (GTK_TABLE (table1), 5);

  label1 = gtk_label_new ("Azimuth");
  gtk_widget_show (label1);
  gtk_table_attach (GTK_TABLE (table1), label1, 0, 1, 1, 2,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (label1), 0, 0.5);

  label2 = gtk_label_new ("Elevation");
  gtk_widget_show (label2);
  gtk_table_attach (GTK_TABLE (table1), label2, 0, 1, 2, 3,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (label2), 0, 0.5);

  label3 = gtk_label_new ("Id");
  gtk_widget_show (label3);
  gtk_table_attach (GTK_TABLE (table1), label3, 0, 1, 3, 4,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (label3), 0, 0.5);

  label4 = gtk_label_new ("mode");
  gtk_widget_show (label4);
  gtk_table_attach (GTK_TABLE (table1), label4, 0, 1, 0, 1,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (label4), 0, 0.5);

  radiobutton1 = gtk_radio_button_new_with_mnemonic (NULL, "manual");
  gtk_widget_show (radiobutton1);
  gtk_table_attach (GTK_TABLE (table1), radiobutton1, 1, 2, 0, 1,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (radiobutton1), radiobutton1_group);
  radiobutton1_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (radiobutton1));

  radiobutton2 = gtk_radio_button_new_with_mnemonic (NULL, "tracking");
  gtk_widget_show (radiobutton2);
  gtk_table_attach (GTK_TABLE (table1), radiobutton2, 2, 3, 0, 1,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (radiobutton2), radiobutton1_group);
  radiobutton1_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (radiobutton2));

  azim_scale = gtk_hscale_new (GTK_ADJUSTMENT (gtk_adjustment_new (144.7, 0, 360, 1, 1, 1)));
  gtk_widget_show (azim_scale);
  gtk_table_attach (GTK_TABLE (table1), azim_scale, 1, 3, 1, 2,
                    (GtkAttachOptions) (GTK_EXPAND | GTK_FILL),
                    (GtkAttachOptions) (GTK_FILL), 0, 0);
  gtk_range_set_update_policy (GTK_RANGE (azim_scale), GTK_UPDATE_DELAYED);

  elev_scale = gtk_hscale_new (GTK_ADJUSTMENT (gtk_adjustment_new (32.3, 0, 90, 1, 1, 1)));
  gtk_widget_show (elev_scale);
  gtk_table_attach (GTK_TABLE (table1), elev_scale, 1, 3, 2, 3,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (GTK_FILL), 0, 0);

  entry1 = gtk_entry_new ();
  gtk_widget_show (entry1);
  gtk_table_attach (GTK_TABLE (table1), entry1, 1, 3, 3, 4,
                    (GtkAttachOptions) (GTK_EXPAND | GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);

  g_signal_connect ((gpointer) radiobutton1, "toggled",
                    G_CALLBACK (on_mode_changed),
                    NULL);

  /* Store pointers to all widgets, for use by lookup_widget(). */
  GLADE_HOOKUP_OBJECT_NO_REF (window1, window1, "window1");
  GLADE_HOOKUP_OBJECT (window1, vbox1, "vbox1");
  GLADE_HOOKUP_OBJECT (window1, vbox2, "vbox2");
  GLADE_HOOKUP_OBJECT (window1, table1, "table1");
  GLADE_HOOKUP_OBJECT (window1, label1, "label1");
  GLADE_HOOKUP_OBJECT (window1, label2, "label2");
  GLADE_HOOKUP_OBJECT (window1, label3, "label3");
  GLADE_HOOKUP_OBJECT (window1, label4, "label4");
  GLADE_HOOKUP_OBJECT (window1, radiobutton1, "radiobutton1");
  GLADE_HOOKUP_OBJECT (window1, radiobutton2, "radiobutton2");
  GLADE_HOOKUP_OBJECT (window1, entry1, "entry1");

  return window1;


}

void on_ANTENNA_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  //  azim_sp = atof(argv[0]);
  //  elev_sp = atof(argv[1]);
  //  id_sp = atoi(argv[2]);
  //  mode = atoi(argv[3]);

  if (mode == AUTO) {
    azim_sp = atof(argv[0]);
    elev_sp = atof(argv[1]);
    gtk_range_set_value(azim_scale, azim_sp);
    gtk_range_set_value(elev_scale, elev_sp);

  }
  g_message("on antenna status %d", mode);
}

int main (int argc, char** argv) {

  gtk_init(&argc, &argv);

  GtkWidget* window = build_gui();
  gtk_widget_show_all(window);

  IvyInit ("AntennaTracker", "AntennaTracker READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_ANTENNA_STATUS, NULL, "^\\S* ANTENNA_STATUS (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  gtk_main();
  return 0;
}
