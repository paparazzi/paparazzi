#ifndef FMS_NETWORK_H
#define FMS_NETWORK_H

#include <sys/socket.h>
#include <arpa/inet.h>

enum FmsNetworkMode {
  FMS_NETWORK_UNICAST,
  FMS_NETWORK_BROADCAST
};

#define FMS_UNICAST 0
#define FMS_BROADCAST 1

struct FmsNetwork {
  int socket_in;
  int socket_out;
  struct sockaddr_in addr_in;
  struct sockaddr_in addr_out;
};


extern struct FmsNetwork* network_new(const char* str_ip_out, const int port_out, const int port_in, 
				      enum FmsNetworkMode mode);
extern int network_write(struct FmsNetwork* me, char* buf, int len);

#endif /* FMS_NETWORK_H */

