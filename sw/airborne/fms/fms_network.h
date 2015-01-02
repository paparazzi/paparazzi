#ifndef FMS_NETWORK_H
#define FMS_NETWORK_H

#include <sys/socket.h>
#include <arpa/inet.h>

#define FMS_UNICAST 0
#define FMS_BROADCAST 1

struct FmsNetwork {
  int socket_in;
  int socket_out;
  struct sockaddr_in addr_in;
  struct sockaddr_in addr_out;
};


extern struct FmsNetwork *network_new(const char *str_ip_out, const int port_out, const int port_in,
                                      const int broadcast);
extern int network_subscribe_multicast(struct FmsNetwork *me, const char *multicast_addr);
extern int network_set_recvbuf(struct FmsNetwork *me, int buf_size);
extern int network_write(struct FmsNetwork *me, char *buf, int len);
extern int network_read(struct FmsNetwork *me, unsigned char *buf, int len);

#endif /* FMS_NETWORK_H */
