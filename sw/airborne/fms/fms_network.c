#include "fms_network.h"



#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>

struct FmsNetwork* network_new(const char* str_ip, const int port) {

  struct FmsNetwork* me = malloc(sizeof(struct FmsNetwork));
 
  // struct FgNetChannel* chan = malloc(sizeof (struct FgNetChannel));
  int so_reuseaddr = 1;
  struct protoent * pte = getprotobyname("UDP");
  me->fd = socket( PF_INET, SOCK_DGRAM, pte->p_proto);
  setsockopt(me->fd, SOL_SOCKET, SO_REUSEADDR, 
             &so_reuseaddr, sizeof(so_reuseaddr));

  me->addr.sin_family = PF_INET;
  me->addr.sin_port = htons(port);
  me->addr.sin_addr.s_addr = inet_addr(str_ip);
  
  return me;

}
