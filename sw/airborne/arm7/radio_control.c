#include "radio_control.h"
#include "traces.h"


int32_t rc_values[RADIO_CTL_NB];
int32_t avg_last_radio[RADIO_CTL_NB];
uint8_t last_radio_contains_avg_channels = FALSE;
uint8_t rc_status;
uint8_t time_sime_last_ppm;

void radio_control_init ( void ) {
  rc_status = RC_REALLY_LOST; 
  time_sime_last_ppm = RC_REALLY_LOST_TIME;
}

void radio_control_periodic_task ( void ) {
  if (time_sime_last_ppm >= RC_REALLY_LOST_TIME)
    rc_status = RC_REALLY_LOST;
  else {
    time_sime_last_ppm++;
    if (time_sime_last_ppm >= RC_LOST_TIME)
      rc_status = RC_LOST;
  }
}

void radio_control_process_ppm ( void ) {
  time_sime_last_ppm = 0;
  rc_status = RC_OK;
  NORMALISE_PPM();
  static uint32_t foo;
  foo++;
  if (!(foo%10)) {
    PRINT_RADIO_CONTROL();
  }
}
