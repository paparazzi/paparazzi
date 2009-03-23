
#include <glib.h>

#include <stdio.h>

#define AP_DEVICE "/dev/ttyACM0"
#define GS_IP     "192.168.1.8"
#define GS_PORT   2442

#include "fms_ap_link.h"
#include "fms_gs_link.h"


static gboolean on_ap_link_data_received(GIOChannel *source,
					 GIOCondition condition,
					 gpointer data);

static gboolean on_gs_link_data_received(GIOChannel *source,
					 GIOCondition condition,
					 gpointer data);


static gboolean on_ap_link_data_received(GIOChannel *source,
					 GIOCondition condition,
					 gpointer data) {
  struct FmsApLink* ap_link = (struct FmsApLink*)data;
  gsize bytes_read;
  GError* _err = NULL;
  GIOStatus st = g_io_channel_read_chars(source, ap_link->buf, AP_LINK_BUF_SIZE, &bytes_read, &_err);
  printf("in on_ap_link_data_received %d %d\n", st, _err);
  if (_err != NULL) {
    fprintf (stderr, "error reading serial: %s\n", _err->message);
    g_error_free (_err);

  }
  if (st == G_IO_STATUS_NORMAL) {
    ap_link_parse(ap_link, bytes_read);
  }
  return TRUE;
}

static gboolean on_gs_link_data_received(GIOChannel *source,
					 GIOCondition condition,
					 gpointer data) {
  return TRUE;
}


int main(int argc, char** argv) {

  struct FmsApLink* ap_link;
  struct FmsGsLink* gs_link;
  ap_link = ap_link_new(AP_DEVICE);
  if (!ap_link) {
    printf("error opening serial port %s\n", AP_DEVICE);
    return -1;
  }
  GIOChannel* ioc1 = g_io_channel_unix_new(ap_link->sp->fd);
  g_io_channel_set_encoding(ioc1, NULL, NULL);
  g_io_add_watch (ioc1, G_IO_IN, on_ap_link_data_received, ap_link);

  //  gs_link = gs_link_new(GS_IP, GS_PORT);
  //  GIOChannel* ioc2 = g_io_channel_unix_new(gs_link->network->fd);
  //  g_io_add_watch (ioc2, G_IO_IN, on_gs_link_data_received, ap_link);

  GMainLoop* ml = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(ml);

  return 0;
}
