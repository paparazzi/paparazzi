#ifndef BOOZ_GPS_UBX_H
#define BOOZ_GPS_UBX_H

#include "ubx_protocol.h"

#define GPS_UBX_MAX_PAYLOAD 255
struct BoosGpsUbx {
  bool_t  msg_available;
  uint8_t msg_buf[GPS_UBX_MAX_PAYLOAD] __attribute__ ((aligned));
  uint8_t msg_id;
  uint8_t msg_class;

  uint8_t  status;
  uint16_t len;
  uint8_t  msg_idx;
  uint8_t  ck_a, ck_b;
  uint8_t  error_cnt;
  uint8_t  error_last;
};

extern struct BoosGpsUbx booz_gps_ubx;

extern void booz_gps_ubx_read_message(void);
extern void booz_gps_ubx_parse(uint8_t c);

#define BoozGpsEvent(_sol_available_callback) {				\
    if (GpsBuffer()) {							\
      ReadGpsBuffer();							\
    }									\
    if (booz_gps_ubx.msg_available) {					\
      booz_gps_ubx_read_message();					\
      if (booz_gps_ubx.msg_class == UBX_NAV_ID &&			\
      booz_gps_ubx.msg_id == UBX_NAV_SOL_ID) {			\
        if (booz_gps_state.fix == BOOZ2_GPS_FIX_3D)                     \
          booz_gps_state.lost_counter = 0;				\
    _sol_available_callback();					\
      }									\
      booz_gps_ubx.msg_available = FALSE;				\
    }									\
  }

#define ReadGpsBuffer() {					\
    while (GpsLink(ChAvailable())&&!booz_gps_ubx.msg_available)	\
      booz_gps_ubx_parse(GpsLink(Getch()));			\
  }

#endif /* BOOZ_GPS_UBX_H */
