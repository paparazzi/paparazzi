#ifndef SOCKET_H
#define SOCKET_H


#include <sys/socket.h>
#include <arpa/inet.h>

#define FMS_UNICAST 0
#define FMS_BROADCAST 1

struct UdpSocket {
  int socket_in;
  int socket_out;
  struct sockaddr_in addr_in;
  struct sockaddr_in addr_out;
};


extern struct UdpSocket *udp_socket(const char *str_ip_out, const int port_out, const int port_in, const int broadcast);
extern int udp_write(struct UdpSocket *me, unsigned char *buf, int len);
extern int udp_read(struct UdpSocket *me, unsigned char *buf, int len);


#endif  /*  SOCKET_H  */

