#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

//#include "sliding_plot.h"

#define MB_MODES_IDLE   0
#define MB_MODES_MANUAL 1
#define MB_MODES_RAMP   2
#define MB_MODES_STEP   3
#define MB_MODES_PRBS   4

const guint mb_id = 145;


static void on_mode_changed (GtkRadioButton  *radiobutton, gpointer user_data);


GtkWidget* build_gui ( void ) {

  GtkWidget *window1;
  GtkWidget *vbox1;


  window1 = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window1), "motor_bench");

  vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window1), vbox1);

  GSList *rb_mode_group = NULL;
  GtkWidget* rb_idle = gtk_radio_button_new_with_mnemonic (NULL, "idle");
  gtk_box_pack_start (GTK_BOX (vbox1), rb_idle, TRUE, TRUE, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (rb_idle), rb_mode_group);
  rb_mode_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (rb_idle));
  GtkWidget* rb_manual = gtk_radio_button_new_with_mnemonic (NULL, "manual");
  gtk_box_pack_start (GTK_BOX (vbox1), rb_manual, TRUE, TRUE, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (rb_manual), rb_mode_group);
  rb_mode_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (rb_manual));
  GtkWidget* rb_ramp = gtk_radio_button_new_with_mnemonic (NULL, "ramp");
  gtk_box_pack_start (GTK_BOX (vbox1), rb_ramp, TRUE, TRUE, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (rb_ramp), rb_mode_group);
  rb_mode_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (rb_ramp));
  GtkWidget* rb_step = gtk_radio_button_new_with_mnemonic (NULL, "step");
  gtk_box_pack_start (GTK_BOX (vbox1), rb_step, TRUE, TRUE, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (rb_step), rb_mode_group);
  rb_mode_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (rb_step));
  GtkWidget* rb_prbs = gtk_radio_button_new_with_mnemonic (NULL, "prbs");
  gtk_box_pack_start (GTK_BOX (vbox1), rb_prbs, TRUE, TRUE, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (rb_prbs), rb_mode_group);
  rb_mode_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (rb_prbs));

  g_signal_connect ((gpointer) rb_idle, "toggled", G_CALLBACK (on_mode_changed), (gpointer)MB_MODES_IDLE);
  g_signal_connect ((gpointer) rb_manual, "toggled", G_CALLBACK (on_mode_changed), (gpointer)MB_MODES_MANUAL);
  g_signal_connect ((gpointer) rb_ramp, "toggled", G_CALLBACK (on_mode_changed), (gpointer)MB_MODES_RAMP);
  g_signal_connect ((gpointer) rb_step, "toggled", G_CALLBACK (on_mode_changed), (gpointer)MB_MODES_STEP);
  g_signal_connect ((gpointer) rb_prbs, "toggled", G_CALLBACK (on_mode_changed), (gpointer)MB_MODES_PRBS);

  return window1;
}

static void on_mode_changed (GtkRadioButton  *radiobutton, gpointer user_data) {
  if (!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radiobutton)))
    return;
  guint mode = (guint)user_data;
  IvySendMsg("1ME RAW_DATALINK %d SETTING;0;0;%d", mb_id, mode);
  g_message("on mode changed %d" , mode);
}



#if 0
void on_scale_value_changed (GtkScale  *scale, gpointer user_data) {
  gfloat cf = gtk_range_get_value(GTK_RANGE(scale));
  gint c = round(cf);
  g_message("foo %d %f", user_data, c);
  IvySendMsg("ME RAW_DATALINK 16 SETTING;%d;0;%d", (gint)user_data, c);
}
#endif


void on_MOTOR_BENCH_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  int time_ticks =  atoi(argv[0]);
  g_message("foo %d", time_ticks);

}

int main (int argc, char** argv) {

  gtk_init(&argc, &argv);

  GtkWidget* window = build_gui();
  gtk_widget_show_all(window);

  IvyInit ("MotorBench", "MotorBench READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_MOTOR_BENCH_STATUS, NULL, "^\\S* MOTOR_BENCH_STATUS (\\S*) (\\S*) (\\S*) (\\S*),(\\S*),(\\S*)");
  IvyStart("127.255.255.255");

  gtk_main();
  return 0;
}
