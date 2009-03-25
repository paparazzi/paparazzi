#include "fms_gs_link.h"

#include <stdlib.h>

#include "fms_debug.h"

struct FmsGsLink* gs_link_new(const char* str_ip, const int port) {

  struct FmsGsLink* me = malloc(sizeof(struct FmsGsLink));
  me->network = network_new(str_ip, port);
  return me;

}



int gs_link_write(struct FmsGsLink* me, char* buf, int len) {

  TRACE(TRACE_DEBUG, "gs_link writing %d bytes to network\n", len);
  return network_write(me->network, buf, len);

}


