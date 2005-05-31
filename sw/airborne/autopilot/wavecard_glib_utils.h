#ifndef WAVECARD_GLIB_UTILS_H
#define WAVECARD_GLIB_UTILS_H

#include <glib.h>
#include <inttypes.h>

#include "wavecard.h"

struct WavecardAddr {
  uint8_t addr[WC_ADDR_LEN];
  gchar* name;
};

struct WavecardAddr* get_host_by_addr ( uint8_t* addr);
struct WavecardAddr* get_host_by_name ( uint8_t* name);
struct WavecardAddr* get_host_by_id   ( uint8_t  id);
uint8_t              get_nb_hosts     ();

GIOChannel* open_serial_port( const gchar* serial_dev);
void print_both(gsize len, guchar* buf);
void wc_glib_print_addr(uint8_t* addr);
void wc_glib_append_addr(GString* str, uint8_t* addr);




#endif /* WAVECARD_GLIB_UTILS_H */
