
#include "fms_gs_link.h"

#include <stdlib.h>

struct FmsGsLink* gs_link_new(const char* str_ip, const int port) {

  struct FmsGsLink* me = malloc(sizeof(struct FmsGsLink));
  me->network = network_new(str_ip, port);
  return me;

}

