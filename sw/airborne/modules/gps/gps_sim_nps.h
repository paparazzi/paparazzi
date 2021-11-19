#ifndef GPS_SIM_NPS_H
#define GPS_SIM_NPS_H

#include "std.h"

#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_SIM
#endif

extern struct GpsState gps_nps;
extern bool gps_has_fix;

extern void gps_feed_value();

extern void gps_nps_init();

#define gps_nps_periodic_check() gps_periodic_check(&gps_nps)

#endif /* GPS_SIM_NPS_H */
