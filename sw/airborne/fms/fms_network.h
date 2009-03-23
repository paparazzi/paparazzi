#ifndef FMS_NETWORK_H
#define FMS_NETWORK_H

#include <sys/socket.h>
#include <arpa/inet.h>

struct FmsNetwork {
  int fd;
  struct sockaddr_in addr;
};


extern struct FmsNetwork* network_new(const char* str_ip, const int port);

#endif /* FMS_NETWORK_H */

