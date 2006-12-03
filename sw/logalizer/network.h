#ifndef NETWORK_H
#define NETWORK_H

#include <inttypes.h>
#include <sys/socket.h>
#include <arpa/inet.h>

struct FgNetChannel {
  int socket;
  struct sockaddr_in addr;
};

extern struct FgNetChannel* open_out_channel(  char* host, uint16_t port );
extern struct FgNetChannel* open_in_channel(  char* host, uint16_t port );
extern void send_buf(struct FgNetChannel* chan, char* buf, int len);


#endif /* NETWORK_H */
