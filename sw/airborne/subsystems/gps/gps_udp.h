#ifndef GPS_UDP_H
#define GPS_UDP_H

#include "std.h"
#include "subsystems/gps.h"

#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_UDP
#endif

extern struct GpsState gps_udp;

extern void gps_udp_parse(void);
extern void gps_udp_init(void);

#endif /* GPS_UDP_H */
