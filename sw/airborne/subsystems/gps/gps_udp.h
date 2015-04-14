#ifndef GPS_UDP_H
#define GPS_UDP_H

#include "std.h"

#define GPS_NB_CHANNELS 16

extern void gps_parse(void);

#define GpsEvent gps_parse

#endif /* GPS_UDP_H */
