#include "fms_network.h"

#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <errno.h>

#include "fms_debug.h"

struct FmsNetwork* network_new(const char* str_ip_out, const int port_out, const int port_in, const int broadcast) {

  struct FmsNetwork* me = malloc(sizeof(struct FmsNetwork));

  int so_reuseaddr = 1;
  struct protoent * pte = getprotobyname("UDP");
  me->socket_out = socket( PF_INET, SOCK_DGRAM, pte->p_proto);
  setsockopt(me->socket_out, SOL_SOCKET, SO_REUSEADDR,
             &so_reuseaddr, sizeof(so_reuseaddr));

  /* only set broadcast option if explicitly enabled */
  if (broadcast)
    setsockopt(me->socket_out, SOL_SOCKET, SO_BROADCAST,
               &broadcast, sizeof(broadcast));

  me->addr_out.sin_family = PF_INET;
  me->addr_out.sin_port = htons(port_out);
  me->addr_out.sin_addr.s_addr = inet_addr(str_ip_out);

  me->socket_in = socket( PF_INET, SOCK_DGRAM, pte->p_proto);
  setsockopt(me->socket_in, SOL_SOCKET, SO_REUSEADDR,
             &so_reuseaddr, sizeof(so_reuseaddr));

  me->addr_in.sin_family = PF_INET;
  me->addr_in.sin_port = htons(port_in);
  me->addr_in.sin_addr.s_addr = htonl(INADDR_ANY);

  bind(me->socket_in, (struct sockaddr *)&me->addr_in, sizeof(me->addr_in));

  return me;

}

int network_write(struct FmsNetwork* me, char* buf, int len) {
  ssize_t byte_written = sendto(me->socket_out, buf, len, MSG_DONTWAIT,
                                (struct sockaddr*)&me->addr_out, sizeof(me->addr_out));
  if ( byte_written != len) {
    TRACE(TRACE_ERROR, "error sending to network %d (%d)\n", byte_written, errno);
  }
  return len;
}

///< returns: -1 = error, 0 = no data, >0 = nrofbytesread

int network_read(struct FmsNetwork* me, unsigned char* buf, int len) {

  socklen_t slen = sizeof(struct sockaddr_in);
  // MSG_DONTWAIT => nonblocking flag
  ssize_t byte_read = recvfrom(me->socket_in, buf, len, MSG_DONTWAIT,
                                (struct sockaddr*)&me->addr_in, &slen);

  if (byte_read == -1) {
    if (errno == EWOULDBLOCK) { // If not data is available, simply return zero bytes read, no error
      return 0;
    }
    else {
      TRACE(TRACE_ERROR, "error reading from network error %d \n",errno);
    }
  }

  return byte_read;
}
