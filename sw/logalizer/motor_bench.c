#include <glib.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <gtk/gtk.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "sliding_plot.h"

GtkWidget *throttle_plot;
GtkWidget *rpm_plot;

GtkWidget *manual_throttle_scale;

GSList *radiobutton_group = NULL;

#define PARAM_TIME     0
#define PARAM_THROTTLE 1
#define PARAM_RPM      2
#define PARAM_VOLTAGE  3
#define PARAM_CURRENT  4
#define PARAM_THRUST   5
#define PARAM_TORQUE   6
#define PARAM_NB       7

const gchar* param_names[] = {
  "Time",
  "Throttle",
  "RPM",
  "Voltage",
  "Current",
  "Thrust",
  "Torque"
};

GtkWidget* param_label[PARAM_NB];
gfloat param_value[PARAM_NB];

#define MODE_HALTED 0
#define MODE_MANUAL 1
#define MODE_RAMP   2
#define MODE_STEP   3
#define MODE_PRBS   4
#define MODE_NB     5

guint mode;

GtkWidget* build_gui ( void );

gboolean timeout_callback(gpointer data) {
  GString* str = g_string_sized_new(64);
  g_string_printf(str, "%.1f", param_value[PARAM_TIME]);
  gtk_label_set_text(GTK_LABEL(param_label[PARAM_TIME]), str->str);
  g_string_printf(str, "%.1f", param_value[PARAM_THROTTLE]);
  gtk_label_set_text(GTK_LABEL(param_label[PARAM_THROTTLE]), str->str);
  g_string_free(str, TRUE);
  return TRUE;
}

void on_mode_changed (GtkRadioButton  *radiobutton, gpointer user_data) {
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radiobutton))) {
    guint new_mode =  (gint)user_data;
    IvySendMsg("ME RAW_DATALINK 78 SETTING;0;0;%d", new_mode);
    //    IvySendMsg("ME DL_SETTINGS 78 0 %d.0", new_mode);
    g_message("sending new mode %d" , new_mode);  
  }
}

void on_manual_throttle_value_changed (GtkScale  *scale, gpointer user_data) {
  gfloat new_throttle = gtk_range_get_value(GTK_RANGE(scale));
  g_message("foo %f", new_throttle);
  guint foo = (new_throttle/100.*9600.);
  IvySendMsg("ME RAW_DATALINK 78 SETTING;3;0;%d", foo);
}


#define TICK_PER_SEC 15000000.
void on_MOTOR_BENCH_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  guint time_tick = atoi(argv[0]);
  guint time_sec = atoi(argv[1]);
  guint throttle = atoi(argv[2]);
  guint new_mode = atoi(argv[3]);

  float time = (float)time_sec + (float)time_tick / TICK_PER_SEC;
  param_value[PARAM_TIME] = time;
  param_value[PARAM_THROTTLE] = (float)throttle / 9600. * 100.;
  //  printf("%f %d %d\n", time, throttle, mode);
  gfloat foo[] = {(gfloat)throttle};
  sliding_plot_update(throttle_plot, foo);

  if (new_mode != mode) {
    g_message("mode changed %d->%d", mode, new_mode);  
    GtkRadioButton  *radiobutton = g_slist_nth(radiobutton_group, MODE_NB-new_mode-1)->data;
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(radiobutton), TRUE);
    mode = new_mode;
  }
}


int main ( int argc, char** argv) {

  gtk_init(&argc, &argv);
  
  g_timeout_add(160, timeout_callback, NULL);

  //  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  
  IvyInit ("MotorBench", "MotorBench READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_MOTOR_BENCH_STATUS, NULL, "^\\S* MOTOR_BENCH_STATUS (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  //  g_main_loop_run(ml);

  GtkWidget* window = build_gui();
  gtk_widget_show_all(window);

  gtk_main();

  return 0;
}



GtkWidget* build_gui ( void ) {
  GtkWidget *window;
  GtkWidget *vbox1;
  GtkWidget *table1;
  GtkWidget *label4;

  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window), "motor bench");

  vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_widget_show (vbox1);
  gtk_container_add (GTK_CONTAINER (window), vbox1);

  table1 = gtk_table_new (4, 3, FALSE);
  gtk_widget_show (table1);
  gtk_box_pack_start (GTK_BOX (vbox1), table1, TRUE, TRUE, 0);
  gtk_table_set_col_spacings (GTK_TABLE (table1), 5);

  label4 = gtk_label_new ("mode");
  gtk_widget_show (label4);
  gtk_table_attach (GTK_TABLE (table1), label4, 0, 1, 0, 1,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (label4), 0, 0.5);

  gchar* mode_names[] = { "Halted", "Manual", "Ramp", "Step", "PRBS" };

  gint i;
  for (i=0; i<MODE_NB; i++) {
    GtkWidget* radiobutton = gtk_radio_button_new_with_mnemonic (NULL, mode_names[i]);
    gtk_widget_show (radiobutton);
    gtk_table_attach (GTK_TABLE (table1), radiobutton, 0, 1, 1+i, 2+i,
		      (GtkAttachOptions) (GTK_FILL),
		      (GtkAttachOptions) (0), 0, 0);
    gtk_radio_button_set_group (GTK_RADIO_BUTTON (radiobutton), radiobutton_group);
    radiobutton_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (radiobutton));
    g_signal_connect ((gpointer) radiobutton, "toggled",
		      G_CALLBACK (on_mode_changed),
		      ((gpointer)i));
  }

  manual_throttle_scale = gtk_hscale_new (GTK_ADJUSTMENT (gtk_adjustment_new (0., 0, 100, 1, 1, 1)));
  gtk_widget_show (manual_throttle_scale);
  gtk_table_attach (GTK_TABLE (table1), manual_throttle_scale, 1, 1+PARAM_NB, 1+MODE_MANUAL, 2+MODE_MANUAL,
                    (GtkAttachOptions) (GTK_EXPAND | GTK_FILL),
                    (GtkAttachOptions) (GTK_FILL), 0, 0);
  gtk_range_set_update_policy (GTK_RANGE (manual_throttle_scale), GTK_UPDATE_DELAYED);
  g_signal_connect ((gpointer) manual_throttle_scale, "value_changed",
                    G_CALLBACK (on_manual_throttle_value_changed),
                    NULL);



  for (i=0; i<PARAM_NB; i++) {
    GtkWidget* label = gtk_label_new (param_names[i]);
    gtk_table_attach (GTK_TABLE (table1), label, i, i+1, PARAM_NB+1, PARAM_NB + 2,
		      (GtkAttachOptions) (GTK_FILL),
		      (GtkAttachOptions) (0), 0, 0);
    param_label[i] = gtk_label_new ("unknown");
    gtk_table_attach (GTK_TABLE (table1), param_label[i], i, i+1, PARAM_NB+2, PARAM_NB + 3,
		      (GtkAttachOptions) (GTK_FILL),
		      (GtkAttachOptions) (0), 0, 0);
  }

  GtkWidget *frame = gtk_frame_new ("Throttle");
 
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);
  throttle_plot = sliding_plot_new(1);
  gtk_container_add (GTK_CONTAINER (frame), throttle_plot );
 

  frame = gtk_frame_new ("RPM");
 
  gtk_container_set_border_width (GTK_CONTAINER (frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);
  //  rpm_plot = sliding_plot_new(1);
  //  gtk_container_add (GTK_CONTAINER (frame), rpm_plot );

 
 gtk_box_pack_start (GTK_BOX (vbox1), frame, TRUE, TRUE, 0);
 

 return window;
}
