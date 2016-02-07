#ifndef GPS_SIM_NPS_H
#define GPS_SIM_NPS_H

#include "std.h"

//#define GPS_NB_CHANNELS 16

#define PrimaryGpsImpl nps

extern bool_t gps_has_fix;

extern void gps_feed_value();

extern void nps_gps_impl_init();
extern void nps_gps_event(void);
extern void nps_gps_register(void);


#endif /* GPS_SIM_NPS_H */
