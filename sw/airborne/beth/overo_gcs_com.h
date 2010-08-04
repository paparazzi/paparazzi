#ifndef OVERO_GCS_COM_H
#define OVERO_GCS_COM_H

#include "fms_network.h"
#include "downlink_transport.h"

#include "std.h"

#define GCS_COM_DL_BUF_SIZE 128

struct OveroGcsCom {

  struct FmsNetwork* network;
  struct DownlinkTransport *udp_transport;
  /*  bool_t  my_dl_msg_available; */
  uint8_t my_dl_buffer[GCS_COM_DL_BUF_SIZE]  __attribute__ ((aligned));

};


extern struct OveroGcsCom gcs_com;


extern void gcs_com_init(char* gcs_host, uint32_t gcs_port, uint32_t datalink_port, uint8_t broadcast);
extern void gcs_com_periodic(void);


#endif /* OVERO_GCS_COM_H */
