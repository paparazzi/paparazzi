#ifndef GPS_SIM_H
#define GPS_SIM_H

#include "std.h"
#include "subsystems/gps.h"

#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_SIM
#endif

extern void gps_sim_publish(void);

extern void gps_sim_init(void);

#endif /* GPS_SIM_H */
