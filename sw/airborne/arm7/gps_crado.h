#ifndef GPS_CRADO_H
#define GPS_CRADO_H

#include "types.h"
#include "ubx.h"
#include "ubx_protocol.h"

extern uint8_t gps_mode;
extern uint32_t gps_itow;    /* ms */
extern int32_t  gps_alt;    /* cm       */
extern uint16_t gps_gspeed;  /* cm/s     */
extern int16_t  gps_climb;  /* m/s     */
extern int16_t  gps_course; /* decideg     */
extern int32_t gps_utm_east, gps_utm_north; /** cm */
extern uint8_t gps_utm_zone;

void gps_init( void );
void parse_gps_msg( void );
void parse_ubx( uint8_t c );
extern volatile uint8_t gps_msg_received;
extern uint8_t gps_pos_available;
extern uint8_t gps_nb_ovrn;

#define NB_CHANNELS 16

/** Number of scanned satellites */
extern uint8_t gps_nb_channels;

/** Space Vehicle Information */
struct svinfo {
  uint8_t svid;
  uint8_t flags;
  uint8_t qi;
  uint8_t cno;
  int8_t elev; /** deg */
  int16_t azim; /** deg */
};

extern struct svinfo gps_svinfos[NB_CHANNELS];


#endif /* GPS_CRADO_H */
