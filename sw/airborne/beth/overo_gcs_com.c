#include "overo_gcs_com.h"

#include <event.h>
#include <string.h>
#include <unistd.h>

#include "downlink_transport.h"
#include "messages2.h"
#include "udp_transport2.h"
#include "dl_protocol.h"
#include "settings.h"
//#include "downlink.h"


static void dl_handle_msg(struct DownlinkTransport *tp);
static inline int checked_read(int fd, char *buf, size_t len);
static void on_datalink_event(int fd, short event __attribute__((unused)), void *arg);

struct OveroGcsCom gcs_com;


void gcs_com_init(char* gcs_host, uint32_t gcs_port, uint32_t datalink_port, uint8_t broadcast) {

  gcs_com.network = network_new(gcs_host, gcs_port, datalink_port, broadcast);
  gcs_com.udp_transport = udp_transport_new(gcs_com.network);
  struct event datalink_event;
  event_set(&datalink_event, gcs_com.network->socket_in, EV_READ| EV_PERSIST, on_datalink_event, gcs_com.udp_transport);
  event_add(&datalink_event, NULL);
  
}

void gcs_com_periodic(void) {
  if (gcs_com.udp_transport->Periodic) 
    gcs_com.udp_transport->Periodic(gcs_com.udp_transport->impl);
}


static void on_datalink_event(int fd, short event __attribute__((unused)), void *arg) {

  char buf[255];
  int bytes_read;
  bytes_read = checked_read(fd, buf, sizeof(buf) - 1);
  struct DownlinkTransport *tp = (struct DownlinkTransport *) arg;
  struct udp_transport *udp_impl = tp->impl;
  printf("on datalink event: %d bytes\n",bytes_read);
  int i = 0;
  while (i<bytes_read) {
    parse_udp_dl(udp_impl, buf[i]);
    i++;
    if (udp_impl->udp_dl_msg_received) {
      memcpy(gcs_com.my_dl_buffer, udp_impl->udp_dl_payload, udp_impl->udp_dl_payload_len);
      dl_handle_msg(tp);
      udp_impl->udp_dl_msg_received = FALSE;
    }
  }
 
}

#define IdOfMsg(x) (x[1])

static void dl_handle_msg(struct DownlinkTransport *tp) {
  uint8_t msg_id = IdOfMsg(gcs_com.my_dl_buffer);
  switch (msg_id) {
  
  case  DL_PING:
    DOWNLINK_SEND_PONG(tp);
    break;
    
  case DL_SETTING :
    {
      uint8_t i = DL_SETTING_index(gcs_com.my_dl_buffer);
      float var = DL_SETTING_value(gcs_com.my_dl_buffer);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(tp, &i, &var);
    }
    break;

  default :
    printf("did nothing\n");
    break;
  }

}


static inline int checked_read(int fd, char *buf, size_t len)
{
  int bytes = read(fd, buf, len);

  if (bytes == 0) {
    fprintf(stderr, "Connection closed\n");
  } else if (bytes == -1) {
    perror("read");
  }
  return bytes;
}


