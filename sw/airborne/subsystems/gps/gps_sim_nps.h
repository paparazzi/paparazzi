#ifndef GPS_SIM_NPS_H
#define GPS_SIM_NPS_H

#include "nps_sensors.h"

#define GPS_LINKChAvailable() (FALSE)
#define GPS_LINKGetch() (TRUE)

#define GPS_NB_CHANNELS 16

extern bool_t gps_available;

extern void gps_feed_value();

extern void gps_impl_init();

#define GpsEvent(_sol_available_callback) {     \
    if (gps_available) {                        \
      if (gps.fix == GPS_FIX_3D)                \
        gps.lost_counter = 0;                   \
      _sol_available_callback();				\
      gps_available = FALSE;                    \
    }                                           \
  }

#endif /* GPS_SIM_NPS_H */
