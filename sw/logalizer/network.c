#include "network.h"

#include <netdb.h>
#include <netinet/in.h>

#include <stdlib.h>
#include <stdio.h>


struct FgNetChannel* open_out_channel( char* host, uint16_t port) {
  struct FgNetChannel* chan = malloc(sizeof (struct FgNetChannel));
  int so_reuseaddr = 1;
  struct protoent * pte = getprotobyname("UDP");
  chan->socket = socket( PF_INET, SOCK_DGRAM, pte->p_proto);
  setsockopt(chan->socket, SOL_SOCKET, SO_REUSEADDR,
	     &so_reuseaddr, sizeof(so_reuseaddr));

  chan->addr.sin_family = PF_INET;
  chan->addr.sin_port = htons(port);
  chan->addr.sin_addr.s_addr = inet_addr(host);

  return chan;
}

struct FgNetChannel* open_in_channel( char* host, uint16_t port) {
  struct FgNetChannel* chan = open_out_channel(host, port);
  int ret = bind(chan->socket, (struct sockaddr*)&chan->addr, sizeof(chan->addr));
  if (ret)
    perror("binding\n");
  return chan;
}


void send_buf(struct FgNetChannel* chan, char* buf, int len) {
  if (sendto(chan->socket, buf, len, 0,
             (struct sockaddr*)&chan->addr, sizeof(chan->addr)) == -1)
    printf("error sending\n");
}
