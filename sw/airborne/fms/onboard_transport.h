#ifndef ONBOARD_TRANSPORT_H
#define ONBOARD_TRANSPORT_H

#include "std.h"
#include "generated/airframe.h"

#ifndef MSG_TIMESTAMP
#define MSG_TIMESTAMP 0
#endif

#define ONBOARD_BUFFER_LEN 1500

struct DownlinkTransport *onboard_transport_new(char *path, uint32_t *timestamp);

struct onboard_transport {
  int fd;
  char buffer[ONBOARD_BUFFER_LEN];
  uint32_t buffer_idx;
  uint32_t array_length;
  uint32_t *timestamp;
  uint32_t overrun;
};

#endif /* ONBOARD_TRANSPORT_H */

