#ifndef GPS_UDP_H
#define GPS_UDP_H

#include "std.h"

#define UDP_GPS_NB_CHANNELS 16

#ifndef PrimaryGpsImpl
#define PrimaryGpsImpl udp
#endif


extern void gps_parse(void);
extern void udp_gps_impl_init(void);
extern void udp_gps_register(void);


// #define GpsEvent gps_parse

#endif /* GPS_UDP_H */
