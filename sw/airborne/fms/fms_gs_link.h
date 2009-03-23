#ifndef FMS_GS_LINK_H
#define FMS_GS_LINK_H

#include "fms_network.h"

struct FmsGsLink {
  struct FmsNetwork* network;
};

extern struct FmsGsLink* gs_link_new(const char* str_ip, const int port);



#endif /* FMS_GS_LINK_H */
