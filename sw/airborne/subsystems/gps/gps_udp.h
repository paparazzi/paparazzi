#ifndef GPS_UDP_H
#define GPS_UDP_H

#include "std.h"

#define UDP_GPS_NB_CHANNELS 16

#ifndef PRIMARY_GPS
#define PRIMARY_GPS gps_udp
#endif

extern void gps_udp_parse(void);
extern void gps_udp_init(void);
extern void gps_udp_register(void);

#endif /* GPS_UDP_H */
