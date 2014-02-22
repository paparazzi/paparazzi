#ifndef GPS_SIM_H
#define GPS_SIM_H

#include "std.h"

#define GPS_NB_CHANNELS 16

extern bool_t gps_available;

//extern void gps_feed_values(double utm_north, double utm_east, double utm_alt, double gspeed, double course, double climb);

extern void gps_impl_init(void);

#define GpsEvent(_sol_available_callback) {         \
    if (gps_available) {                            \
      gps.last_msg_ticks = sys_time.nb_sec_rem;     \
      gps.last_msg_time = sys_time.nb_sec;          \
      if (gps.fix == GPS_FIX_3D) {                  \
        gps.last_3dfix_ticks = sys_time.nb_sec_rem; \
        gps.last_3dfix_time = sys_time.nb_sec;      \
      }                                             \
      _sol_available_callback();                    \
      gps_available = FALSE;                        \
    }                                               \
  }

#endif /* GPS_SIM_H */
