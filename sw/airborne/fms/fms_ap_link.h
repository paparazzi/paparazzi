#ifndef FMS_AP_LINK_H
#define FMS_AP_LINK_H

#include "fms_serial_port.h"

#define AP_LINK_BUF_SIZE 4096

struct FmsApLink {
  struct FmsSerialPort* sp;
  char buf[AP_LINK_BUF_SIZE];
};

extern struct FmsApLink* ap_link_new(const char* device);
extern void ap_link_free(struct FmsApLink* me);
extern void ap_link_parse(struct FmsApLink* me, int nb_bytes);

#endif /* FMS_AP_LINK_H */
