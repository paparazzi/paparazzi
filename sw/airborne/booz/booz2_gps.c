#include "booz2_gps.h"

#include "led.h"

struct Booz_gps_state booz_gps_state;

/* UBX NAV POSLLH */
int32_t  booz2_gps_lon;
int32_t  booz2_gps_lat;
int32_t  booz2_gps_hmsl;
uint32_t booz2_gps_hacc;
uint32_t booz2_gps_vacc;

/* UBX NAV VELNED */
int32_t  booz2_gps_vel_n;
int32_t  booz2_gps_vel_e;


/* misc */
volatile bool_t  booz2_gps_msg_received;
volatile uint8_t booz2_gps_nb_ovrn;

void booz2_gps_init(void) {

  booz_gps_state.fix = BOOZ2_GPS_FIX_NONE;
#ifdef GPS_LED
  LED_ON(GPS_LED);
#endif
  ubx_init();

}


void booz2_gps_read_ubx_message(void) {

  if (ubx_class == UBX_NAV_ID) {
    if (ubx_id == UBX_NAV_POSLLH_ID) {
      booz2_gps_lon = UBX_NAV_POSLLH_LON(ubx_msg_buf);
      booz2_gps_lat = UBX_NAV_POSLLH_LAT(ubx_msg_buf);
      booz2_gps_hmsl = UBX_NAV_POSLLH_HMSL(ubx_msg_buf);
    }
    else if (ubx_id == UBX_NAV_SOL_ID) {
      booz_gps_state.fix          = UBX_NAV_SOL_GPSfix(ubx_msg_buf);
      booz_gps_state.ecef_pos.x   = UBX_NAV_SOL_ECEF_X(ubx_msg_buf);
      booz_gps_state.ecef_pos.y   = UBX_NAV_SOL_ECEF_Y(ubx_msg_buf);
      booz_gps_state.ecef_pos.z   = UBX_NAV_SOL_ECEF_Z(ubx_msg_buf);
      booz_gps_state.pacc         = UBX_NAV_SOL_Pacc(ubx_msg_buf);
      booz_gps_state.ecef_speed.x = UBX_NAV_SOL_ECEFVX(ubx_msg_buf);
      booz_gps_state.ecef_speed.y = UBX_NAV_SOL_ECEFVY(ubx_msg_buf);
      booz_gps_state.ecef_speed.z = UBX_NAV_SOL_ECEFVZ(ubx_msg_buf);
      booz_gps_state.sacc         = UBX_NAV_SOL_Sacc(ubx_msg_buf);
      booz_gps_state.pdop         = UBX_NAV_SOL_PDOP(ubx_msg_buf);
      booz_gps_state.num_sv       = UBX_NAV_SOL_numSV(ubx_msg_buf);
#ifdef GPS_LED
      if (booz_gps_state.fix == BOOZ2_GPS_FIX_3D) {
      	LED_OFF(GPS_LED);
      }
      else {
	LED_TOGGLE(GPS_LED);
      }
#endif
    }
    else if (ubx_id == UBX_NAV_VELNED_ID) {
      booz2_gps_vel_n = UBX_NAV_VELNED_VEL_N(ubx_msg_buf);
      booz2_gps_vel_e = UBX_NAV_VELNED_VEL_E(ubx_msg_buf);
    }
  }

}

/* UBX parsing */

bool_t  ubx_msg_available;
#define UBX_MAX_PAYLOAD 255
uint8_t ubx_msg_buf[UBX_MAX_PAYLOAD] __attribute__ ((aligned));
uint8_t ubx_id;
uint8_t ubx_class;

#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_CLASS     3
#define GOT_ID        4
#define GOT_LEN1      5
#define GOT_LEN2      6
#define GOT_PAYLOAD   7
#define GOT_CHECKSUM1 8

static uint8_t  ubx_status;
static uint16_t ubx_len;
static uint8_t  ubx_msg_idx;
static uint8_t  ck_a, ck_b;


void ubx_init(void) {
  ubx_status = UNINIT;
  ubx_msg_available = FALSE;
}

void ubx_parse( uint8_t c ) {
  if (ubx_status < GOT_PAYLOAD) {
    ck_a += c;
    ck_b += ck_a;
  }
  switch (ubx_status) {
  case UNINIT:
    if (c == UBX_SYNC1)
      ubx_status++;
    break;
  case GOT_SYNC1:
    if (c != UBX_SYNC2)
      goto error;
    ck_a = 0;
    ck_b = 0;
    ubx_status++;
    break;
  case GOT_SYNC2:
    if (ubx_msg_available) {
      /* Previous message has not yet been parsed: discard this one */
      booz2_gps_nb_ovrn++;
      goto error;
    }
    ubx_class = c;
    ubx_status++;
    break;
  case GOT_CLASS:
    ubx_id = c;
    ubx_status++;
    break;    
  case GOT_ID:
    ubx_len = c;
    ubx_status++;
    break;
  case GOT_LEN1:
    ubx_len |= (c<<8);
    if (ubx_len > UBX_MAX_PAYLOAD)
      goto error;
    ubx_msg_idx = 0;
    ubx_status++;
    break;
  case GOT_LEN2:
    ubx_msg_buf[ubx_msg_idx] = c;
    ubx_msg_idx++;
    if (ubx_msg_idx >= ubx_len) {
      ubx_status++;
    }
    break;
  case GOT_PAYLOAD:
    if (c != ck_a)
      goto error;
    ubx_status++;
    break;
  case GOT_CHECKSUM1:
    if (c != ck_b)
      goto error;
    ubx_msg_available = TRUE;
    goto restart;
    break;
  }
  return;
 error:  
 restart:
  ubx_status = UNINIT;
  return;
}
