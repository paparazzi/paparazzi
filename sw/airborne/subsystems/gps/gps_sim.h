#ifndef GPS_SIM_H
#define GPS_SIM_H

#include "std.h"

#define GPS_NB_CHANNELS 16

extern void gps_impl_init(void);

extern void gps_sim_publish(void);

#define GpsEvent() {}

#endif /* GPS_SIM_H */
