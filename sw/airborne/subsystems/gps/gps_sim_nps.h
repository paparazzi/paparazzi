#ifndef GPS_SIM_NPS_H
#define GPS_SIM_NPS_H

#include "std.h"

#define GPS_NB_CHANNELS 16

extern bool_t gps_has_fix;

extern void gps_feed_value();

extern void gps_impl_init();

#define GpsEvent() {}

#endif /* GPS_SIM_NPS_H */
