#ifndef GPS_SIM_NPS_H
#define GPS_SIM_NPS_H

#include "std.h"

#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_SIM
#endif

extern bool gps_has_fix;

extern void gps_feed_value();

extern void gps_nps_impl_init();

#endif /* GPS_SIM_NPS_H */
