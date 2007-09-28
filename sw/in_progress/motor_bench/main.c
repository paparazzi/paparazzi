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

struct motor_bench_state {
  guint mode;
  double time;
  double throttle;
  double rpms;
  double amps;
  double thrust;
  double torque;
  GIOChannel* log_channel;
};

struct motor_bench_gui {
  GtkWidget* lab_time;
  GtkWidget* lab_throttle;
  GtkWidget* lab_rpms;
  GtkWidget* lab_amps;
  GtkWidget* lab_thrust;
  GtkWidget* lab_torque;
  GtkWidget* entry_log;
};

static void on_mode_changed (GtkRadioButton  *radiobutton, gpointer user_data);
static void on_MOTOR_BENCH_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]);
static gboolean timeout_callback(gpointer data);
static void on_log_button_toggled (GtkWidget *widget, gpointer data);
static GtkWidget* build_gui ( void );

static struct motor_bench_state mb_state;
static struct motor_bench_gui mb_gui;

static void on_mode_changed (GtkRadioButton  *radiobutton, gpointer user_data) {
  if (!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radiobutton)))
    return;
  guint mode = (guint)user_data;
  IvySendMsg("1ME RAW_DATALINK %d SETTING;0;0;%d", mb_id, mode);
  //  g_message("on mode changed %d" , mode);
}

#if 0
void on_scale_value_changed (GtkScale  *scale, gpointer user_data) {
  gfloat cf = gtk_range_get_value(GTK_RANGE(scale));
  gint c = round(cf);
  g_message("foo %d %f", user_data, c);
  IvySendMsg("ME RAW_DATALINK 16 SETTING;%d;0;%d", (gint)user_data, c);
}
#endif


static void on_MOTOR_BENCH_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  guint time_ticks =  atoi(argv[0]);
  double throttle =  atof(argv[1]);
  double rpm =  atof(argv[2]);
  double amp =  atof(argv[3]);
  double thrust =  atof(argv[4]);
  double torque =  atof(argv[5]);
  guint time_sec = atoi(argv[6]);
  guint mode = atoi(argv[7]);
  mb_state.mode = mode;
  mb_state.time = (double)time_sec + (double)time_ticks/15000000.;
  mb_state.throttle = throttle;
  mb_state.rpms = rpm;
  mb_state.amps = amp;
  mb_state.thrust = thrust;
  mb_state.torque = torque;
  if (mb_state.log_channel) {
    GString* str = g_string_sized_new(256);
    g_string_printf(str, "%.4f %.3f %.0f %.1f %.1f %.1f\n", mb_state.time, mb_state.throttle, mb_state.rpms, mb_state.amps, mb_state.thrust, mb_state.torque);
    gsize b_writen;
    GError* my_err = NULL;
    GIOStatus stat = g_io_channel_write_chars(mb_state.log_channel,str->str, str->len, &b_writen, &my_err); 
    g_string_free(str, TRUE);
  }
  //  g_message("foo %f %f %f %f %d", mb_state.time, throttle, rpm, amp, mode);
}

static gboolean timeout_callback(gpointer data) {
  GString* str = g_string_sized_new(64);
  g_string_printf(str, "%.2f s", mb_state.time);
  gtk_label_set_text(GTK_LABEL(mb_gui.lab_time), str->str);
  g_string_printf(str, "%.2f %%", mb_state.throttle);
  gtk_label_set_text(GTK_LABEL(mb_gui.lab_throttle), str->str);
  g_string_printf(str, "%.0f rpms", mb_state.rpms);
  gtk_label_set_text(GTK_LABEL(mb_gui.lab_rpms), str->str);
  g_string_printf(str, "%.1f a", mb_state.amps);
  gtk_label_set_text(GTK_LABEL(mb_gui.lab_amps), str->str);
  g_string_printf(str, "%.1f g", mb_state.thrust);
  gtk_label_set_text(GTK_LABEL(mb_gui.lab_thrust), str->str);
  g_string_printf(str, "%.1f g", mb_state.torque);
  gtk_label_set_text(GTK_LABEL(mb_gui.lab_torque), str->str);
  g_string_free(str, TRUE);
  return TRUE;
}

static void on_log_button_toggled (GtkWidget *widget, gpointer data) {
   if (gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON (widget))) {
     gtk_editable_set_editable( GTK_EDITABLE(mb_gui.entry_log), FALSE );
     const gchar *log_file_name = gtk_entry_get_text (GTK_ENTRY (mb_gui.entry_log));
     GError* my_err = NULL;
     mb_state.log_channel = g_io_channel_new_file (log_file_name, "w", &my_err);
   }
   else {
     gtk_editable_set_editable( GTK_EDITABLE(mb_gui.entry_log), TRUE );
     if (mb_state.log_channel) {
       g_io_channel_close(mb_state.log_channel);
       mb_state.log_channel = NULL;
     }
   }
}

int main (int argc, char** argv) {

  gtk_init(&argc, &argv);

  GtkWidget* window = build_gui();
  gtk_widget_show_all(window);

  IvyInit ("MotorBench", "MotorBench READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_MOTOR_BENCH_STATUS, NULL, "^\\S* MOTOR_BENCH_STATUS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  g_timeout_add(40, timeout_callback, NULL);
  
  mb_state.log_channel = NULL;

  gtk_main();
  return 0;
}


static GtkWidget* build_gui ( void ) {

  GtkWidget *window1;
  GtkWidget *vbox1;


  window1 = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window1), "motor_bench");

  vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window1), vbox1);

  GtkWidget* hbox1 = gtk_hbox_new (FALSE, 0);
  gtk_box_pack_start (GTK_BOX (vbox1), hbox1, TRUE, TRUE, 0);

  //
  // Modes
  //
  GtkWidget *mode_frame = gtk_frame_new ("Mode");
  gtk_container_set_border_width (GTK_CONTAINER (mode_frame), 10);
  gtk_box_pack_start (GTK_BOX (hbox1), mode_frame, TRUE, TRUE, 0);

  GtkWidget* vbox2 = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (mode_frame), vbox2);

  GSList *rb_mode_group = NULL;
  GtkWidget* rb_idle = gtk_radio_button_new_with_mnemonic (NULL, "idle");
  gtk_box_pack_start (GTK_BOX (vbox2), rb_idle, TRUE, TRUE, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (rb_idle), rb_mode_group);
  rb_mode_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (rb_idle));
  GtkWidget* rb_manual = gtk_radio_button_new_with_mnemonic (NULL, "manual");
  gtk_box_pack_start (GTK_BOX (vbox2), rb_manual, TRUE, TRUE, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (rb_manual), rb_mode_group);
  rb_mode_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (rb_manual));
  GtkWidget* rb_ramp = gtk_radio_button_new_with_mnemonic (NULL, "ramp");
  gtk_box_pack_start (GTK_BOX (vbox2), rb_ramp, TRUE, TRUE, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (rb_ramp), rb_mode_group);
  rb_mode_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (rb_ramp));
  GtkWidget* rb_step = gtk_radio_button_new_with_mnemonic (NULL, "step");
  gtk_box_pack_start (GTK_BOX (vbox2), rb_step, TRUE, TRUE, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (rb_step), rb_mode_group);
  rb_mode_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (rb_step));
  GtkWidget* rb_prbs = gtk_radio_button_new_with_mnemonic (NULL, "prbs");
  gtk_box_pack_start (GTK_BOX (vbox2), rb_prbs, TRUE, TRUE, 0);
  gtk_radio_button_set_group (GTK_RADIO_BUTTON (rb_prbs), rb_mode_group);
  rb_mode_group = gtk_radio_button_get_group (GTK_RADIO_BUTTON (rb_prbs));

  g_signal_connect ((gpointer) rb_idle, "toggled", G_CALLBACK (on_mode_changed), (gpointer)MB_MODES_IDLE);
  g_signal_connect ((gpointer) rb_manual, "toggled", G_CALLBACK (on_mode_changed), (gpointer)MB_MODES_MANUAL);
  g_signal_connect ((gpointer) rb_ramp, "toggled", G_CALLBACK (on_mode_changed), (gpointer)MB_MODES_RAMP);
  g_signal_connect ((gpointer) rb_step, "toggled", G_CALLBACK (on_mode_changed), (gpointer)MB_MODES_STEP);
  g_signal_connect ((gpointer) rb_prbs, "toggled", G_CALLBACK (on_mode_changed), (gpointer)MB_MODES_PRBS);


  //
  // Measures
  //
  GtkWidget *measure_frame = gtk_frame_new ("Measures");
  gtk_container_set_border_width (GTK_CONTAINER (measure_frame), 10);
  gtk_box_pack_start (GTK_BOX (hbox1), measure_frame, TRUE, TRUE, 0);

  GtkWidget* table1 = gtk_table_new (2, 3, FALSE);
  gtk_container_add (GTK_CONTAINER (measure_frame), table1);
  gtk_table_set_col_spacings (GTK_TABLE (table1), 5);
  
  GtkWidget* t_time = gtk_label_new ("time");
  gtk_table_attach (GTK_TABLE (table1), t_time, 0, 1, 0, 1,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (t_time), 0, 0.5);
  mb_gui.lab_time = gtk_label_new ("XXXX");
  gtk_table_attach (GTK_TABLE (table1), mb_gui.lab_time, 1, 2, 0, 1,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (mb_gui.lab_time), 0, 0.5);

  GtkWidget* t_throttle = gtk_label_new ("throttle");
  gtk_table_attach (GTK_TABLE (table1), t_throttle, 0, 1, 1, 2,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (t_throttle), 0, 0.5);
  mb_gui.lab_throttle = gtk_label_new ("XXXX");
  gtk_table_attach (GTK_TABLE (table1), mb_gui.lab_throttle, 1, 2, 1, 2,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (mb_gui.lab_throttle), 0, 0.5);

  GtkWidget* t_rpms = gtk_label_new ("rpms");
  gtk_table_attach (GTK_TABLE (table1), t_rpms, 0, 1, 2, 3,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (t_rpms), 0, 0.5);
  mb_gui.lab_rpms = gtk_label_new ("XXXX");
  gtk_table_attach (GTK_TABLE (table1), mb_gui.lab_rpms, 1, 2, 2, 3,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (mb_gui.lab_rpms), 0, 0.5);

  GtkWidget* t_amps = gtk_label_new ("amps");
  gtk_table_attach (GTK_TABLE (table1), t_amps, 0, 1, 3, 4,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (t_amps), 0, 0.5);
  mb_gui.lab_amps = gtk_label_new ("XXXX");
  gtk_table_attach (GTK_TABLE (table1), mb_gui.lab_amps, 1, 2, 3, 4,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (mb_gui.lab_amps), 0, 0.5);

  GtkWidget* t_thrust = gtk_label_new ("Thrust");
  gtk_table_attach (GTK_TABLE (table1), t_thrust, 0, 1, 4, 5,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (t_thrust), 0, 0.5);
  mb_gui.lab_thrust = gtk_label_new ("XXXX");
  gtk_table_attach (GTK_TABLE (table1), mb_gui.lab_thrust, 1, 2, 4, 5,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (mb_gui.lab_thrust), 0, 0.5);

  GtkWidget* t_torque = gtk_label_new ("Torque");
  gtk_table_attach (GTK_TABLE (table1), t_torque, 0, 1, 5, 6,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (t_torque), 0, 0.5);
  mb_gui.lab_torque = gtk_label_new ("XXXX");
  gtk_table_attach (GTK_TABLE (table1), mb_gui.lab_torque, 1, 2, 5, 6,
                    (GtkAttachOptions) (GTK_FILL),
                    (GtkAttachOptions) (0), 0, 0);
  gtk_misc_set_alignment (GTK_MISC (mb_gui.lab_torque), 0, 0.5);


  //
  // Log
  //
  GtkWidget *log_frame = gtk_frame_new ("Log");
  gtk_container_set_border_width (GTK_CONTAINER (log_frame), 10);
  gtk_box_pack_start (GTK_BOX (vbox1), log_frame, TRUE, TRUE, 0);
  

  GtkWidget* bbox = gtk_hbutton_box_new (); 
  gtk_container_add (GTK_CONTAINER (log_frame), bbox);

  GtkWidget* log_button = gtk_toggle_button_new_with_label( "Log" );
  gtk_container_add (GTK_CONTAINER (bbox), log_button);
  g_signal_connect (G_OBJECT (log_button), "toggled", G_CALLBACK (on_log_button_toggled), NULL);

  mb_gui.entry_log = gtk_entry_new( );
  gtk_container_add (GTK_CONTAINER (bbox), mb_gui.entry_log);
  gtk_entry_set_text( GTK_ENTRY(mb_gui.entry_log), "mb_log.txt" );
  return window1;
}
