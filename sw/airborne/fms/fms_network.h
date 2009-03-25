#ifndef FMS_NETWORK_H
#define FMS_NETWORK_H

#include <sys/socket.h>
#include <arpa/inet.h>

struct FmsNetwork {
  int socket;
  struct sockaddr_in addr;
};


extern struct FmsNetwork* network_new(const char* str_ip, const int port);
extern int network_write(struct FmsNetwork* me, char* buf, int len);

#endif /* FMS_NETWORK_H */

