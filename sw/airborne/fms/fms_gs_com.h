/*
 * This module handles communications with ground segment
 *
 * for now one channel of telemetry/datalink via wifi
 *
 */

#ifndef FMS_GS_COM_H
#define FMS_GS_COM_H

#include <event.h>
#include "fms_network.h"
#include "downlink_transport.h"

#include "std.h"

struct FmsGsCom {

  struct FmsNetwork *network;
  struct DownlinkTransport *udp_transport;
  struct event datalink_event;

};


extern struct FmsGsCom fms_gs_com;
/* remove me */
extern uint8_t telemetry_mode_Main_DefaultChannel;

extern uint8_t fms_gs_com_init(const char *gs_host, uint16_t gs_port,
                               uint16_t datalink_port, uint8_t broadcast);
extern void fms_gs_com_periodic(void);

#endif /* FMS_GS_COM_H */
