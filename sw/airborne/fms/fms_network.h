#ifndef FMS_NETWORK_H
#define FMS_NETWORK_H

#include <sys/socket.h>
#include <arpa/inet.h>

struct FmsNetwork {
  int socket_in;
  int socket_out;
  struct sockaddr_in addr_in;
  struct sockaddr_in addr_out;
};


extern struct FmsNetwork* network_new(const char* str_ip_out, const int port_out, const int port_in);
extern int network_write(struct FmsNetwork* me, char* buf, int len);

#endif /* FMS_NETWORK_H */

