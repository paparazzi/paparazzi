#ifndef GPS_SIM_NPS_H
#define GPS_SIM_NPS_H

#include "std.h"

#ifndef PRIMARY_GPS
#define PRIMARY_GPS gps_nps
#endif

extern bool gps_has_fix;

extern void gps_feed_value();

extern void gps_nps_impl_init();
extern void gps_nps_register(void);


#endif /* GPS_SIM_NPS_H */
