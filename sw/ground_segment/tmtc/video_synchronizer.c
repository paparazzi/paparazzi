#include <glib.h>
#include <gtk/gtk.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>



#define DEBUGMESSAGES FALSE



float startVideoAfter;
float logCurrentTime;
float currentPlayingTime;
GtkWidget *spinButton;
GMainLoop *ml;

//function taken from the mplayer project
void send_udp(const char *send_to_ip, int port, char *mesg)
{
  static int sockfd = -1;
  static struct sockaddr_in socketinfo;

  if (sockfd == -1) {
    static const int one = 1;
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

  sendto(sockfd, mesg, strlen(mesg), 0, (struct sockaddr *) &socketinfo,
         sizeof(socketinfo));
}


void sendCurrentPlayingTime(void)
{
  char current_time[256];

  sprintf(current_time, "%f", currentPlayingTime);

#if DEBUGMESSAGES
  g_print("sendding value : %s\n", current_time);
#endif

  send_udp("127.0.0.1", 23867, current_time);
}


void calcCurrentPlayingTime(void)
{
  float syncTime;

#if DEBUGMESSAGES
  g_print("log : %f; start : %f\n", logCurrentTime, startVideoAfter);
#endif

  syncTime = logCurrentTime-startVideoAfter;
  if(syncTime < 0.0) syncTime = 0.0;

  currentPlayingTime = syncTime;
}

static void on_Message(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  logCurrentTime = atof(argv[1]);

  calcCurrentPlayingTime();
  sendCurrentPlayingTime();
}


static gboolean __timeout_func(gpointer data)
{
#if DEBUGMESSAGES
  g_print("timeout_callback\n");
#endif

  sendCurrentPlayingTime();
  return TRUE;
}


static void on_quit(GtkWidget *object, gpointer   user_data)
{
  gtk_main_quit();
}


static void on_video_sync_changed(GtkWidget *widget, gpointer data)
{
  startVideoAfter = gtk_spin_button_get_value_as_float((GtkSpinButton *)spinButton);

  calcCurrentPlayingTime();
  sendCurrentPlayingTime();
}



int main ( int argc, char** argv)
{
  currentPlayingTime = 0.0;
  startVideoAfter = 0.0;

  ml =  g_main_loop_new(NULL, FALSE);
  g_timeout_add (1000 , __timeout_func , ml);

  IvyInit("Video Synchronizer", "Video Synchronizer READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_Message, NULL, "^time(\\S*) (\\S*)");

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

  GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window), "Video synchronizer");
  g_signal_connect(GTK_OBJECT(window), "destroy", G_CALLBACK(on_quit), NULL);

  //create a box to align the widgets
  //false -> unheaven sizes; 0 space between widgets
  GtkWidget *box = gtk_hbox_new (FALSE, 0);

  //add a label to explain the values to enter
  GtkWidget *label = gtk_label_new("How long the video must be started \nafter the log file (may be negative) : ");
  gtk_box_pack_start (GTK_BOX (box), label, FALSE, FALSE, 0);
  gtk_widget_show (label);

  //add a spinbutton (to enter the value of the video synchronisation)
  //startValue, lower, upper, step_increment, page_increment, page_size
  GtkAdjustment *adj = (GtkAdjustment *) gtk_adjustment_new (0.0, -999999999999.99, 999999999999.99, 0.1, 1.0, 0.0);
  spinButton = gtk_spin_button_new(adj, 0.1, 1);
  g_signal_connect(GTK_OBJECT (adj), "value_changed", G_CALLBACK(on_video_sync_changed), NULL);
  gtk_box_pack_start (GTK_BOX (box), spinButton, TRUE, TRUE, 0);
  gtk_widget_show (spinButton);

  //add the unit label at the end of the window
  label = gtk_label_new("s");
  gtk_box_pack_start (GTK_BOX (box), label, FALSE, FALSE, 0);
  gtk_widget_show (label);

  gtk_container_add (GTK_CONTAINER (window), box);

  gtk_widget_show_all(window);
  gtk_main();

  return 0;
}
