// nc -l -u -p 2442 > /tmp/test123

#include <glib.h>

#include <stdio.h>

#define AP_DEVICE "/dev/ttyACM0"
#define GS_IP     "10.31.4.19"
#define GS_PORT   2442

#include "fms_debug.h"
#include "fms_ap_link.h"
#include "fms_gs_link.h"


static struct FmsApLink* ap_link;
static struct FmsGsLink* gs_link;


static gboolean on_ap_link_data_received(GIOChannel *source,
					 GIOCondition condition,
					 gpointer data);

static gboolean on_gs_link_data_received(GIOChannel *source,
					 GIOCondition condition,
					 gpointer data);


static gboolean on_ap_link_data_received(GIOChannel *source,
					 GIOCondition condition,
					 gpointer data) {
  gsize bytes_read;
  GError* _err = NULL;
  GIOStatus st = g_io_channel_read_chars(source, ap_link->buf, AP_LINK_BUF_SIZE, &bytes_read, &_err);
  if (!_err) {
    if (st == G_IO_STATUS_NORMAL) {
      ap_link_parse(ap_link, bytes_read);
      gs_link_write(gs_link, ap_link->buf, bytes_read);
    }
  }
  else {
    TRACE(TRACE_ERROR,"error reading serial: %s\n", _err->message);
    g_error_free (_err);
  }
  return TRUE;
}

static gboolean on_gs_link_data_received(GIOChannel *source,
					 GIOCondition condition,
					 gpointer data) {
  return TRUE;
}


int main(int argc, char** argv) {

  ap_link = ap_link_new(AP_DEVICE);
  if (!ap_link) {
    printf("error opening serial port %s\n", AP_DEVICE);
    return -1;
  }
  GIOChannel* ioc1 = g_io_channel_unix_new(ap_link->sp->fd);
  g_io_channel_set_encoding(ioc1, NULL, NULL);
  g_io_add_watch (ioc1, G_IO_IN, on_ap_link_data_received, NULL);

  gs_link = gs_link_new(GS_IP, GS_PORT);
  if (!gs_link) {
    printf("error opening network connection (%s:%d)\n", GS_IP, GS_PORT);
    return -1;
  }
  GIOChannel* ioc2 = g_io_channel_unix_new(gs_link->network->socket);
  g_io_add_watch (ioc2, G_IO_IN, on_gs_link_data_received, NULL);

  GMainLoop* ml = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(ml);

  return 0;
}
