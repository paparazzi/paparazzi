#include "socket.h"

#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>


#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <netdb.h>
#include <errno.h>
#include <netinet/in.h>

# define ADDR_SIZE_TYPE socklen_t
# define SOCKET_ERROR -1
# define IO_SOCKET ioctl

struct UdpSocket* udp_socket(const char* str_ip_out, const int port_out, const int port_in, const int broadcast)
{

  struct UdpSocket* me = malloc(sizeof(struct UdpSocket));

  int so_reuseaddr = 1;
  me->socket_out = socket( AF_INET, SOCK_DGRAM, 0);
  setsockopt(me->socket_out, SOL_SOCKET, SO_REUSEADDR,
             &so_reuseaddr, sizeof(so_reuseaddr));

  /* only set broadcast option if explicitly enabled */
  if (broadcast)
    setsockopt(me->socket_out, SOL_SOCKET, SO_BROADCAST,
               &broadcast, sizeof(broadcast));

  me->addr_out.sin_family = AF_INET;
  me->addr_out.sin_port = htons(port_out);
  me->addr_out.sin_addr.s_addr = inet_addr(str_ip_out);

  me->socket_in = socket( AF_INET, SOCK_DGRAM, 0);
  setsockopt(me->socket_in, SOL_SOCKET, SO_REUSEADDR,
             &so_reuseaddr, sizeof(so_reuseaddr));

  me->addr_in.sin_family = AF_INET;
  me->addr_in.sin_port = htons(port_in);
  me->addr_in.sin_addr.s_addr = htonl(INADDR_ANY);

  bind(me->socket_in, (struct sockaddr *)&me->addr_in, sizeof(me->addr_in));

  return me;
}

#include <stdio.h>
//#define UDP_MODE  MSG_DONTWAIT
#define UDP_MODE 0

int udp_write(struct UdpSocket* me, unsigned char* buf, int len) {
  sendto(me->socket_out, buf, len, UDP_MODE,
                                (struct sockaddr*)&me->addr_out, sizeof(me->addr_out));
  //printf("sendto ret=%d\n",ret);
  return len;
}

unsigned long MIN(unsigned long a, unsigned long b);
unsigned long MIN(unsigned long a, unsigned long b)
{
  if (a<b) return a;
  return b;
}

int udp_read(struct UdpSocket* me, unsigned char* buf, int len)
{
  unsigned long toread = 0;
  int btr = 1;  // set to >0 in order to start the reading loop
  int newbytes = 0;

  int status;

  // if socket is connected
  for (;btr>0;)
  {
    // Check Status
    status = IO_SOCKET(me->socket_in, FIONREAD, &toread);
    if(status == SOCKET_ERROR) {
      printf("problem receiving from socket\n");
      break;
    }

    //printf("UDP has %d bytes\n", toread);
    if (toread <= 0)
      break;

    // If status: ok and new data: read it
    btr = MIN(toread,(unsigned long)len);
    recvfrom(me->socket_in, buf, btr,  0, (struct sockaddr*)&me->addr_in, (socklen_t *) sizeof(me->addr_in) );
    newbytes += btr;
  }
  return newbytes;
}
