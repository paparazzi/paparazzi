#include "fms_network.h"

#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <errno.h>

#include "fms_debug.h"

static inline void create_socket(int *my_socket, const int protocol, const int reuse_addr, const int broadcast)
{
  // Create the socket with the correct protocl
  *my_socket = socket(PF_INET, SOCK_DGRAM, protocol);

  // Enable reusing of addres (must be exactly 1)
  if (reuse_addr == 1) {
    setsockopt(*my_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr));
  }

  // Enable broadcasting
  if (broadcast == 1) {
    setsockopt(*my_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
  }
}

struct FmsNetwork *network_new(const char *str_ip_out, const int port_out, const int port_in, const int broadcast)
{

  struct FmsNetwork *me = malloc(sizeof(struct FmsNetwork));

  if (port_out >= 0) {
    // Create the output socket (enable reuse of the address, and broadcast if necessary)
    create_socket(&me->socket_out, 0, 1, broadcast);

    // Setup the output address
    me->addr_out.sin_family = PF_INET;
    me->addr_out.sin_port = htons(port_out);
    me->addr_out.sin_addr.s_addr = inet_addr(str_ip_out);
  }

  if (port_in >= 0) {
    // Creat the input socket (enable reuse of the address, and disable broadcast)
    create_socket(&me->socket_in, 0, 1, 0);

    // Create the input address
    me->addr_in.sin_family = PF_INET;
    me->addr_in.sin_port = htons(port_in);
    me->addr_in.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(me->socket_in, (struct sockaddr *)&me->addr_in, sizeof(me->addr_in));
  }

  return me;
}

int network_subscribe_multicast(struct FmsNetwork *me, const char *multicast_addr)
{
  // Create the request
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(multicast_addr);
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);

  // Send the request
  return setsockopt(me->socket_in, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq));
}

int network_set_recvbuf(struct FmsNetwork *me, int buf_size)
{
  // Set and check
  unsigned int optval_size = 4;
  int buf_ret;
  setsockopt(me->socket_in, SOL_SOCKET, SO_RCVBUF, (char *)&buf_size, optval_size);
  getsockopt(me->socket_in, SOL_SOCKET, SO_RCVBUF, (char *)&buf_ret, &optval_size);

  if (buf_size != buf_ret) {
    return -1;
  }

  return 0;
}

int network_write(struct FmsNetwork *me, char *buf, int len)
{
  // Check if the output address is set
  if (!me->socket_out) {
    return -1;
  }

  ssize_t byte_written = sendto(me->socket_out, buf, len, MSG_DONTWAIT,
                                (struct sockaddr *)&me->addr_out, sizeof(me->addr_out));
  if (byte_written != len) {
    TRACE(TRACE_ERROR, "error sending to network %d (%d)\n", (int)byte_written, errno);
  }
  return len;
}

///< returns: -1 = error, 0 = no data, >0 = nrofbytesread

int network_read(struct FmsNetwork *me, unsigned char *buf, int len)
{
  // Check if the input address is set
  if (!me->socket_in) {
    return -1;
  }

  socklen_t slen = sizeof(struct sockaddr_in);
  // MSG_DONTWAIT => nonblocking flag
  ssize_t byte_read = recvfrom(me->socket_in, buf, len, MSG_DONTWAIT,
                               (struct sockaddr *)&me->addr_in, &slen);

  if (byte_read == -1) {
    if (errno == EWOULDBLOCK) { // If not data is available, simply return zero bytes read, no error
      return 0;
    } else {
      TRACE(TRACE_ERROR, "error reading from network error %d \n", errno);
    }
  }

  return byte_read;
}
