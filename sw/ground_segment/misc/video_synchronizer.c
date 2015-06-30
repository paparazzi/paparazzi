/*
 * Copyright (C) 2014 karlito139
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#define _GNU_SOURCE

#include <glib.h>
#include <gtk/gtk.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>



#define DEBUGMESSAGES FALSE

#define MODE_REPLAY 0
#define MODE_CAPTURE 1

#define MANUAL_REPLAY 0
#define AUTO_REPLAY 1


gboolean videoSyncTagFlag;
float startVideoAfter;
float timeTagInVideo;
float logCurrentTime;
float videoSyncTag;
float currentPlayingTime;
GtkWidget *spinButton;
GtkWidget *spinButtonVideo;
GtkWidget *flashWindow;
GMainLoop *ml;
uint8_t currentMode;
uint8_t replayMode;
uint16_t airframeID;


//function taken from the mplayer project
void send_udp(const char *send_to_ip, int port, char *mesg) {
  static int16_t sockfd = -1;
  static struct sockaddr_in socketinfo;

  if (sockfd == -1) {
    static const uint8_t one = 1;
    int ip_valid = 0;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    /*if (sockfd == -1)
      exit_player(EXIT_ERROR);*/

    // Enable broadcast
    setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));

#if HAVE_WINSOCK2_H
    socketinfo.sin_addr.s_addr = inet_addr(send_to_ip);
    ip_valid = socketinfo.sin_addr.s_addr != INADDR_NONE;
#else
    ip_valid = inet_aton(send_to_ip, &socketinfo.sin_addr);
#endif

    if (!ip_valid) {
      /*mp_msg(MSGT_CPLAYER, MSGL_FATAL, MSGTR_InvalidIP);
        exit_player(EXIT_ERROR);*/
    }

    socketinfo.sin_family = AF_INET;
    socketinfo.sin_port   = htons(port);
  }

  sendto(sockfd, mesg, (size_t)strlen(mesg), 0, (struct sockaddr *) &socketinfo,
         sizeof(socketinfo));
}

int say(const char *message) {
  char *str;

  if (asprintf(&str, "spd-say '%s'", message) == -1) {
    fprintf(stderr, "Could not allocate mem for say string\n");
    return -1;
  }

  int ret = system(str);

  free(str);

  return ret;
}

void sendCurrentPlayingTime(void) {
  if (currentMode == MODE_REPLAY) {
    char current_time[256];

    sprintf(current_time, "%f", currentPlayingTime);

#if DEBUGMESSAGES
    g_print("sendding value : %s\n", current_time);
#endif

    send_udp("127.0.0.1", 23867, current_time);
  }
}


void calcCurrentPlayingTime(void) {
  if (replayMode == MANUAL_REPLAY) {
    float syncTime;

#if DEBUGMESSAGES
    g_print("log : %f; start : %f\n", logCurrentTime, startVideoAfter);
#endif

    syncTime = logCurrentTime-startVideoAfter;
    if (syncTime < 0.0) syncTime = 0.0;

    currentPlayingTime = syncTime;
  } else if (replayMode == AUTO_REPLAY) {
    currentPlayingTime = logCurrentTime+timeTagInVideo-videoSyncTag;
  }
}



static void on_Message(IvyClientPtr app, void *user_data, int argc, char *argv[]) {
  logCurrentTime = atof(argv[1]);

  if (videoSyncTagFlag) {
    videoSyncTag = logCurrentTime;
    videoSyncTagFlag = FALSE;
  }

  calcCurrentPlayingTime();
  sendCurrentPlayingTime();
}

static void on_Message_Video(IvyClientPtr app, void *user_data, int argc, char *argv[]) {
  videoSyncTagFlag = TRUE;
}

static void on_Airframe_ID(IvyClientPtr app, void *user_data, int argc, char *argv[]) {
  airframeID = atoi(argv[0]);
}



static gboolean __timeout_func(gpointer data) {
#if DEBUGMESSAGES
  g_print("timeout_callback\n");
#endif

  sendCurrentPlayingTime();
  return TRUE;
}


static gboolean __timeout_flashing_window(gpointer data) {

  static uint8_t i=0;

  GdkColor black;
  black.red = 0x0000;
  black.green = 0x0000;
  black.blue = 0x0000;

  GdkColor white;
  white.red = 0xFFFF;
  white.green = 0xFFFF;
  white.blue = 0xFFFF;

  if (i<10) {
      if (i%2) gtk_widget_modify_bg(flashWindow, GTK_STATE_NORMAL, &black);
      else gtk_widget_modify_bg(flashWindow, GTK_STATE_NORMAL, &white);
  }
  else {
    gtk_widget_destroy(flashWindow);
    i=0;
    return FALSE;
  }

  i++;

  return TRUE;
}


static void on_quit(GtkWidget *object, gpointer   user_data) {
  gtk_main_quit();
}


static void on_video_sync_changed(GtkWidget *widget, gpointer data) {

  startVideoAfter = gtk_spin_button_get_value_as_float((GtkSpinButton *)spinButton);

  calcCurrentPlayingTime();
  sendCurrentPlayingTime();
}

static void on_video_time_tag_changed(GtkWidget *widget, gpointer data) {

  timeTagInVideo = gtk_spin_button_get_value_as_float((GtkSpinButton *)spinButtonVideo);

  calcCurrentPlayingTime();
  sendCurrentPlayingTime();
}




static void on_sync_clicked(GtkButton *button, gpointer user_data) {

  static uint8_t syncID = 1;
  char *msg;

  if (asprintf(&msg, "Aircraft %d, video synchronisation %d", airframeID, syncID) > 0) {
    say(msg);
    free(msg);
  }
  else {
    fprintf(stderr, "Could not allocate mem for msg string\n");
  }

  IvySendMsg("%d VIDEO_SYNC %d", airframeID, syncID);  //TODO : get the current airframe ID

  // create a flashing window in full screen for the camera to see
  flashWindow = gtk_window_new(GTK_WINDOW_TOPLEVEL);

  GdkColor white;
  white.red = 0xFFFF;
  white.green = 0xFFFF;
  white.blue = 0xFFFF;
  gtk_widget_modify_bg(flashWindow, GTK_STATE_NORMAL, &white);

  gtk_hbox_new(FALSE, 0);
  gtk_window_fullscreen((GtkWindow*)flashWindow);
  gtk_widget_show_all(flashWindow);

  g_timeout_add(100 , __timeout_flashing_window , ml);

  syncID++;
}

static void on_change_mode(GtkNotebook *notebook, GtkWidget *page, guint page_num, gpointer user_data) {

  currentMode = page_num;
}

static void on_change_replay(GtkNotebook *notebook, GtkWidget *page, guint page_num, gpointer user_data) {

  replayMode = page_num;
}

GdkPixbuf *create_pixbuf(const gchar * filename) {

  GdkPixbuf *pixbuf;
  GError *error = NULL;
  pixbuf = gdk_pixbuf_new_from_file(filename, &error);
  if (!pixbuf) {
    fprintf(stderr, "%s\n", error->message);
    g_error_free(error);
  }

  return pixbuf;
}

int main(int argc, char** argv) {
  currentPlayingTime = 0.0;
  startVideoAfter = 0.0;

  airframeID = 1;

  currentMode = MODE_REPLAY;
  replayMode = MANUAL_REPLAY;


  ml =  g_main_loop_new(NULL, FALSE);
  g_timeout_add(1000 , __timeout_func , ml);

  IvyInit("Video Synchronizer", "Video Synchronizer READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Message, NULL, "^time(\\S*) (\\S*)");
  IvyBindMsg(on_Message_Video, NULL, "^(\\S*) VIDEO_SYNC(\\S*) (\\S*)");
  IvyBindMsg(on_Airframe_ID, NULL, "^replay(\\S*) PONG(\\S*) (\\S*)");

#ifdef __APPLE__
  IvyStart("224.255.255.255");
#else
  IvyStart("127.255.255.255");
#endif

  /* Ugly because remove every possibility of translation.
   * But needed to prevent the use of the comma as float separator.
   */
  gtk_disable_setlocale();

  gtk_init(&argc, &argv);

  GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(window), "Video synchronizer");
  g_signal_connect(GTK_OBJECT(window), "destroy", G_CALLBACK(on_quit), NULL);


  GtkNotebook *tab = (GtkNotebook*)gtk_notebook_new();
  g_signal_connect(GTK_OBJECT(tab), "switch-page", G_CALLBACK(on_change_mode), NULL);


  GtkNotebook *tabReplay = (GtkNotebook*)gtk_notebook_new();
  g_signal_connect(GTK_OBJECT(tabReplay), "switch-page", G_CALLBACK(on_change_replay), NULL);

  //Manual mode
  //create a box to align the widgets
  //false -> unheaven sizes; 0 space between widgets
  GtkWidget *box = gtk_hbox_new(FALSE, 0);

  //add a label to explain the values to enter
  GtkWidget *label = gtk_label_new("How long the video must be started \nafter the log file (may be negative) : ");
  gtk_box_pack_start(GTK_BOX(box), label, FALSE, FALSE, 0);
  gtk_widget_show(label);

  //add a spinbutton (to enter the value of the video synchronisation)
  //startValue, lower, upper, step_increment, page_increment, page_size
  GtkAdjustment *adj = (GtkAdjustment *) gtk_adjustment_new(0.0, -999999999999.99, 999999999999.99, 0.1, 1.0, 0.0);
  spinButton = gtk_spin_button_new(adj, 0.1, 1);
  g_signal_connect(GTK_OBJECT(adj), "value_changed", G_CALLBACK(on_video_sync_changed), NULL);
  gtk_box_pack_start(GTK_BOX(box), spinButton, TRUE, TRUE, 0);
  gtk_widget_show(spinButton);

  //add the unit label at the end of the window
  label = gtk_label_new("s");
  gtk_box_pack_start(GTK_BOX(box), label, FALSE, FALSE, 0);
  gtk_widget_show(label);

  gtk_notebook_append_page(tabReplay, box, gtk_label_new("Manual"));


  //Auto mode
  //create a box to align the widgets
  //false -> unheaven sizes; 0 space between widgets
  GtkWidget *boxAuto = gtk_hbox_new(FALSE, 0);

  //add a label to explain the values to enter
  label = gtk_label_new("At what time is the clap in the video : ");
  gtk_box_pack_start(GTK_BOX(boxAuto), label, FALSE, FALSE, 0);
  gtk_widget_show(label);

  //add a spinbutton (to enter the value of the video synchronisation)
  //startValue, lower, upper, step_increment, page_increment, page_size
  GtkAdjustment *adjAuto = (GtkAdjustment *) gtk_adjustment_new(0.0, 0.00, 999999999999.99, 0.1, 1.0, 0.0);
  spinButtonVideo = gtk_spin_button_new(adjAuto, 0.1, 1);
  g_signal_connect(GTK_OBJECT(adjAuto), "value_changed", G_CALLBACK(on_video_time_tag_changed), NULL);
  gtk_box_pack_start(GTK_BOX(boxAuto), spinButtonVideo, TRUE, TRUE, 0);
  gtk_widget_show(spinButtonVideo);

  //add the unit label at the end of the window
  label = gtk_label_new("s");
  gtk_box_pack_start(GTK_BOX(boxAuto), label, FALSE, FALSE, 0);
  gtk_widget_show(label);

  gtk_notebook_append_page(tabReplay, boxAuto, gtk_label_new("Auto"));


  //create the capture page
  GtkWidget *captureBox = gtk_hbox_new(FALSE, 0);

  GtkWidget *button = gtk_button_new_with_label("sync");
  gtk_box_pack_start(GTK_BOX(captureBox), button, FALSE, FALSE, 0);
  g_signal_connect(GTK_OBJECT(button), "clicked", G_CALLBACK(on_sync_clicked), NULL);
  gtk_widget_show(button);


  gtk_notebook_append_page(tab, (GtkWidget*)tabReplay, gtk_label_new("Replay"));
  gtk_notebook_append_page(tab, (GtkWidget*)captureBox, gtk_label_new("Capture"));

  gtk_container_add(GTK_CONTAINER(window), (GtkWidget*)tab);
  gtk_window_set_icon(GTK_WINDOW(window), create_pixbuf("data/pictures/penguin_icon_vid.png"));

  gtk_widget_show_all(window);
  gtk_main();

  return 0;
}

