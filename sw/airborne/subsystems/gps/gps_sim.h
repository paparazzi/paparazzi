#ifndef GPS_SIM_H
#define GPS_SIM_H

#include "std.h"
#include "subsystems/gps.h"

// #define GPS_NB_CHANNELS 16
#ifndef PrimaryGpsImpl
#define PrimaryGpsImpl sim
#endif

extern void gps_sim_publish(void);

extern void sim_gps_event(void);
extern void sim_gps_impl_init(void);
extern void sim_gps_register(void);

// #define GpsEvent() {}

#endif /* GPS_SIM_H */
