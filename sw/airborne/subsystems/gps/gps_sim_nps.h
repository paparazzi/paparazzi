#ifndef GPS_SIM_NPS_H
#define GPS_SIM_NPS_H

#include "nps_sensors.h"

#define GPS_FIX_NONE 0x00
#define GPS_FIX_2D   0x02
#define GPS_FIX_3D   0x03

#define GPS_NB_CHANNELS 16

extern bool_t gps_available;

extern void gps_feed_value();

extern void gps_impl_init();

#define GpsEvent(_sol_available_callback) {     \
    if (gps_available) {                        \
      if (gps.fix == GPS_FIX_3D)                \
        gps.last_fix_time = sys_time.nb_sec;    \
      _sol_available_callback();				\
      gps_available = FALSE;                    \
    }                                           \
  }

#endif /* GPS_SIM_NPS_H */
