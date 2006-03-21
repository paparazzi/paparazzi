#ifndef RADIO_CONTROL_H
#define RADIO_CONTROL_H

#include "led.h"
#include "sys_time.h"
#include "ppm.h"
#include "radio.h"
#include "airframe.h"
#include "paparazzi.h"

#define RC_AVG_PERIOD 8
#define RC_LOST_TIME 10
#define RC_REALLY_LOST_TIME 20

#define RC_OK          0
#define RC_LOST        1
#define RC_REALLY_LOST 2

extern pprz_t rc_values[PPM_NB_PULSES];
extern uint8_t rc_status;
extern pprz_t avg_rc_values[PPM_NB_PULSES];
extern uint8_t rc_values_contains_avg_channels;
extern uint8_t time_sime_last_ppm;

static inline void radio_control_init ( void ) {
  rc_status = RC_REALLY_LOST; 
  time_sime_last_ppm = RC_REALLY_LOST_TIME;
}

static inline bool_t radio_control_periodic_task ( void ) {
  uint8_t last_status = rc_status;
  if (time_sime_last_ppm >= RC_REALLY_LOST_TIME) {
    rc_status = RC_REALLY_LOST;
    LED_OFF(1);
  }
  else {
    time_sime_last_ppm++;
    if (time_sime_last_ppm >= RC_LOST_TIME) {
      rc_status = RC_LOST;
      LED_TOGGLE(1);
    }
  }
  return last_status != rc_status;
}

static inline bool_t radio_control_ppm_event ( void ) {
  if (ppm_valid) {
    time_sime_last_ppm = 0;
    rc_status = RC_OK;
    NormalizePpm();
    LED_ON(1); 
    ppm_valid = FALSE;
    return TRUE;
  }
  return FALSE;
}

#endif /* RADIO_CONTROL_H */
