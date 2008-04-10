#include "gps.h"

#include "6dof.h"

/* in gps_ubx.c */
volatile uint8_t gps_msg_received;
bool_t gps_pos_available;
uint8_t gps_nb_ovrn;


uint8_t gps_mode;
uint32_t  gps_itow;    /* ms */
int32_t   gps_alt;    /* cm       */
uint16_t  gps_gspeed;  /* cm/s     */
int16_t   gps_climb;  /* cm/s     */
int16_t   gps_course; /* decideg     */
int32_t gps_utm_east, gps_utm_north;
uint8_t gps_utm_zone;
int32_t gps_lat, gps_lon;
struct svinfo gps_svinfos[GPS_NB_CHANNELS];
uint8_t gps_nb_channels = 0;
uint16_t gps_PDOP;
uint32_t gps_Pacc, gps_Sacc;
uint8_t gps_numSV;
uint16_t gps_reset;


void parse_gps_msg( void ) {}

void gps_feed_values(double utm_north, double utm_east, double utm_alt, double gspeed, double course, double climb) {

  gps_utm_north = utm_north;
  gps_utm_east = utm_east;
  gps_alt = utm_alt;
  gps_gspeed = gspeed;
  gps_course = course;
  gps_climb = climb;

  gps_pos_available = TRUE;
}
