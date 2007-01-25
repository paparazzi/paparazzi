
#include "std.h"
#include "sys_time.h"
#include "init_hw.h"
#include "interrupt_hw.h"

#include "led.h"

#include "uart.h"
#include "print.h"

#include "adc.h"

#include "ant_v2x.h"

#include "messages.h"
#include "downlink.h"

#include "ant_tracker.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void);
static inline void main_dl_parse_msg( void );

uint8_t track_mode;

int main( void ) {
  main_init();
  LED_OFF(1);
  LED_OFF(2);
  while (1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  led_init();
  uart1_init_tx();
  uart1_init_rx();
  adc_init();
  ant_tracker_init();
  int_enable();
}

static inline void main_event_task( void ) {
 
  if (ant_v2x_data_available) {
    ant_v2x_read_data();
    DOWNLINK_SEND_ANTENNA_DEBUG(&ant_v2x_data.xraw, &ant_v2x_data.yraw, \
                                &ant_v2x_data.xcal, &ant_v2x_data.ycal, \
                                &ant_v2x_data.heading, &ant_v2x_data.magnitude, \
                                &ant_v2x_data.temp, &ant_v2x_data.distor, \
                                &ant_v2x_data.cal_status);
    ant_v2x_data_available = FALSE;
  }

  if (PprzBuffer()) {
    ReadPprzBuffer();
    if (pprz_msg_received) {
      pprz_parse_payload();
      pprz_msg_received = FALSE;
    }
  }
  if (dl_msg_available) {
    main_dl_parse_msg();
    dl_msg_available = FALSE;
  }
}

static inline void main_periodic_task( void ) {
  static uint8_t cnt;
  cnt++;
  if (!(cnt%16)) {
    //    LED_TOGGLE(2);
    DOWNLINK_SEND_ANTENNA_STATUS(&ant_track_azim, &ant_track_elev, &ant_track_id, &ant_track_mode);
  }
  if (!(cnt%4)) {
    ant_tracker_periodic();
  }
}

#include "settings.h"
#include "traffic_info.h"
#define NAV_UTM_EAST0 360285
#define NAV_UTM_NORTH0 4813595
#define NAV_UTM_ZONE0 31

#define IdOfMsg(x) (x[1])
#define MOfCm(_x) (((float)_x)/100.)

static inline void main_dl_parse_msg(void) {
  uint8_t msg_id = IdOfMsg(dl_buffer);
  if (msg_id == DL_SETTING) {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float var = DL_SETTING_value(dl_buffer);
    DlSetting(i, var);
    DOWNLINK_SEND_DL_VALUE(&i, &var);
  }  
  else if (msg_id == DL_ACINFO) {
    LED_TOGGLE(1);
    uint8_t id = DL_ACINFO_ac_id(dl_buffer);
    float ux = MOfCm(DL_ACINFO_utm_east(dl_buffer));
    float uy = MOfCm(DL_ACINFO_utm_north(dl_buffer));
    float a = MOfCm(DL_ACINFO_alt(dl_buffer));
    float c = RadOfDeg(((float)DL_ACINFO_course(dl_buffer))/ 10.);
    float s = MOfCm(DL_ACINFO_speed(dl_buffer));
    SetAcInfo(id, ux, uy, c, a, s);
  }
 else if (msg_id == DL_WIND_INFO) {

 }
 else {
   //    LED_TOGGLE(1);
  }
}
