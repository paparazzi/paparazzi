#ifndef GPS_UDP_H
#define GPS_UDP_H

#include "std.h"
#include "modules/gps/gps.h"

#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_UDP
#endif

extern struct GpsState gps_udp;

extern void gps_udp_parse(void);
extern void gps_udp_init(void);

#define gps_udp_periodic_check() gps_periodic_check(&gps_udp)

#endif /* GPS_UDP_H */
