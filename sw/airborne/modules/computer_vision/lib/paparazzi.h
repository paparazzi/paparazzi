
////////////////////////////////////////////////
// Paparazzi communication interface

#include <pthread.h>    // pthread_create
#include <gst/gst.h>    // gprint

#include "socket.h"
#include "video_message_structs.h"

struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;

inline void paparazzi_message_server_start(void);
inline void paparazzi_message_send(void);

struct UdpSocket *sock;

inline void paparazzi_message_server_start(void)
{
  sock = udp_socket("192.168.1.1", 2000, 2001, FMS_UNICAST);
}


inline void paparazzi_message_send(void)
{
  udp_write(sock, (char *) &gst2ppz, sizeof(gst2ppz));
  int ret = udp_read(sock, (unsigned char *) &ppz2gst, sizeof(ppz2gst));
  printf("read: %d \n", ret);
}


