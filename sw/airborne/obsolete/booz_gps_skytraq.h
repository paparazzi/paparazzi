#ifndef BOOZ_GPS_SKYTRAQ_H
#define BOOZ_GPS_SKYTRAQ_H

/*
 *
*
*/

#define SKYTRAQ_SYNC1 0xA0
#define SKYTRAQ_SYNC2 0xA1

#define SKYTRAQ_SYNC3 0x0D
#define SKYTRAQ_SYNC4 0x0A


#define SKYTRAQ_ID_NAVIGATION_DATA 0XA8

#define SKYTRAQ_NAVIGATION_DATA_FixMode(_payload) (uint8_t) (*((uint8_t*)_payload+2-2))
#define SKYTRAQ_NAVIGATION_DATA_NumSV(_payload)   (uint8_t) (*((uint8_t*)_payload+3-2))

//#define SKYTRAQ_NAVIGATION_DATA_TOW(_payload)     (uint32_t)(_payload[7] + (((uint32_t)_payload[6])<<8) + (((uint32_t)_payload[5])<<16) + (((uint32_t)_payload[4])<<24))
#define SKYTRAQ_NAVIGATION_DATA_TOW(_payload)     __builtin_bswap32(*(uint32_t*)&_payload[ 6-2])
#define SKYTRAQ_NAVIGATION_DATA_LAT(_payload)     __builtin_bswap32(*( int32_t*)&_payload[10-2])
#define SKYTRAQ_NAVIGATION_DATA_LON(_payload)     __builtin_bswap32(*( int32_t*)&_payload[14-2])
#define SKYTRAQ_NAVIGATION_DATA_AEL(_payload)     __builtin_bswap32(*(uint32_t*)&_payload[18-2])
#define SKYTRAQ_NAVIGATION_DATA_ASL(_payload)     __builtin_bswap32(*(uint32_t*)&_payload[22-2])
//#define SKYTRAQ_NAVIGATION_DATA_GDOP(_payload)    __builtin_bswap16(*(uint16_t*)&_payload[26-2])
//#define SKYTRAQ_NAVIGATION_DATA_PDOP(_payload)    __builtin_bswap(*(uint16_t*)&_payload[28-2])

#define SKYTRAQ_NAVIGATION_DATA_ECEFX(_payload)   __builtin_bswap32(*( int32_t*)&_payload[36-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFY(_payload)   __builtin_bswap32(*( int32_t*)&_payload[40-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFZ(_payload)   __builtin_bswap32(*( int32_t*)&_payload[44-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFVX(_payload)  __builtin_bswap32(*( int32_t*)&_payload[48-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFVY(_payload)  __builtin_bswap32(*( int32_t*)&_payload[52-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFVZ(_payload)  __builtin_bswap32(*( int32_t*)&_payload[56-2])



/* last error type */
#define GPS_SKYTRAQ_ERR_NONE         0
#define GPS_SKYTRAQ_ERR_OVERRUN      1
#define GPS_SKYTRAQ_ERR_MSG_TOO_LONG 2
#define GPS_SKYTRAQ_ERR_CHECKSUM     3
#define GPS_SKYTRAQ_ERR_OUT_OF_SYNC  4
#define GPS_SKYTRAQ_ERR_UNEXPECTED   5

#define GPS_SKYTRAQ_MAX_PAYLOAD 255
struct BoozGpsSkytraq {
  uint8_t msg_buf[GPS_SKYTRAQ_MAX_PAYLOAD] __attribute__ ((aligned));
  bool_t  msg_available;
  uint8_t msg_id;

  uint8_t  status;
  uint16_t len;
  uint8_t  msg_idx;
  uint8_t  checksum;
  uint8_t  error_cnt;
  uint8_t  error_last;
};

extern struct BoozGpsSkytraq booz_gps_skytraq;

extern void booz_gps_skytraq_read_message(void);
extern void booz_gps_skytraq_parse(uint8_t c);

//#include "my_debug_servo.h"

#define BoozGpsEvent(_sol_available_callback) {				\
    if (GpsBuffer()) {							\
      ReadGpsBuffer();							\
    }									\
    if (booz_gps_skytraq.msg_available) {				\
      booz_gps_skytraq_read_message();					\
      if (booz_gps_skytraq.msg_id == SKYTRAQ_ID_NAVIGATION_DATA) {	\
        if (booz_gps_state.fix == BOOZ2_GPS_FIX_3D)                     \
          booz_gps_state.lost_counter = 0;				\
    _sol_available_callback();					\
      }									\
      booz_gps_skytraq.msg_available = FALSE;				\
    }									\
  }

#define ReadGpsBuffer() {						\
    while (GpsLink(ChAvailable())&&!booz_gps_skytraq.msg_available)	\
      booz_gps_skytraq_parse(GpsLink(Getch()));				\
  }
#endif /* BOOZ_GPS_SKYTRAQ_H */
