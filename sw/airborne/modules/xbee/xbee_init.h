
/** @file xbee_init.h
 * @brief Read XBee Address Low and High
 *
 */

#ifndef XBEE_INIT_H
#define XBEE_INIT_H

#include "subsystems/datalink/transport.h"

#define XbBuffer() TransportLink(PPRZ_UART, ChAvailable())
#define XbGetch() TransportLink(PPRZ_UART, Getch())
#define XbTransmit(_x) TransportLink(PPRZ_UART, Transmit(_x))

extern void xbee_module_init(void);

#endif
