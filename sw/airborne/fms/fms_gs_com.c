#include "fms/fms_gs_com.h"

#include <unistd.h>

#include "udp_transport2.h"

/* generated : holds PeriodicSendMain */
#include "generated/periodic_telemetry.h"
/* holds the definitions of PERIODIC_SEND_XXX */
#include "overo_test_passthrough_telemetry.h"
/* holds the definitions of DOWNLINK_SEND_XXX */
#include "messages2.h"

#include "dl_protocol.h"
/* generated : holds DlSetting() and PeriodicSendDlValue() */
#include "generated/settings.h"

struct FmsGsCom fms_gs_com;
uint8_t telemetry_mode_Main_DefaultChannel;

#define PERIODIC_SEND_DL_VALUE(_chan) PeriodicSendDlValue(_chan)

static void on_datalink_event(int fd, short event __attribute__((unused)), void *arg);
static void on_datalink_message(void);

uint8_t fms_gs_com_init(const char *gs_host, uint16_t gs_port,
                        uint16_t datalink_port, uint8_t broadcast)
{

  fms_gs_com.network = network_new(gs_host, gs_port, datalink_port, broadcast);
  fms_gs_com.udp_transport = udp_transport_new(fms_gs_com.network);
  event_set(&fms_gs_com.datalink_event, fms_gs_com.network->socket_in, EV_READ | EV_PERSIST,
            on_datalink_event, fms_gs_com.udp_transport);
  event_add(&fms_gs_com.datalink_event, NULL);

  return 0;
}

void fms_gs_com_periodic(void)
{

  PeriodicSendMain(fms_gs_com.udp_transport);

  RunOnceEvery(10, {fms_gs_com.udp_transport->Periodic(fms_gs_com.udp_transport->impl);});

}


static void on_datalink_event(int fd, short event __attribute__((unused)), void *arg)
{
  char buf[512];
  int bytes_read = read(fd, buf, 512);
  uint16_t i = 0;
  struct udp_transport *tp = fms_gs_com.udp_transport->impl;
  while (i < bytes_read) {
    parse_udp_dl(tp, buf[i]);
    if (tp->udp_dl_msg_received) {
      on_datalink_message();
      tp->udp_dl_msg_received = FALSE;
    }
    i++;
  }

}

static void on_datalink_message(void)
{

  struct udp_transport *tp = fms_gs_com.udp_transport->impl;
  uint8_t msg_id = tp->udp_dl_payload[1];

  switch (msg_id) {
    case  DL_PING:
      DOWNLINK_SEND_PONG(fms_gs_com.udp_transport);
      break;
    case DL_SETTING :  {
      uint8_t i = DL_SETTING_index(tp->udp_dl_payload);
      float var = DL_SETTING_value(tp->udp_dl_payload);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(fms_gs_com.udp_transport, &i, &var);
    }
    break;

    default :
      break;
  }

}

