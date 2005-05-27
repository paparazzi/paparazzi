#ifndef WAVECARD_GLIB_H
#define WAVECARD_GLIB_H

#include <glib.h>
#include <stdio.h>

#include "wavecard.h"

struct WcGlibOptions {
  gchar*    serial_device;
  gboolean  silent;
};

struct WcGlib {
  GMainLoop* ml;
  GIOChannel* in_ch;
  uint8_t radio_param;
  uint8_t self_addr[WC_ADDR_LEN];
  uint8_t* distant_addr;
  struct WcGlibOptions options;
};

GIOChannel* in_ch;

extern struct WcGlib wc_glib;

#define  WcPut1CtlByte(_byte) { \
    int bytes_written; \
    uint8_t b = _byte; \
    g_io_channel_write_chars (wc_glib.in_ch, &b, 1, &bytes_written, NULL); \
    g_io_channel_flush(wc_glib.in_ch, NULL); \
}

#endif /* WAVECARD_GLIB_H */
