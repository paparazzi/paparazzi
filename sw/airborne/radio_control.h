#ifndef RADIO_CONTROL_H
#define RADIO_CONTROL_H

#include "led.h"
#include "sys_time.h"
#include "ppm.h"
#include "radio.h"
#include "paparazzi.h"

#define RC_AVG_PERIOD 8
#define RC_LOST_TIME 10
#define RC_REALLY_LOST_TIME 20

#define RC_OK          0
#define RC_LOST        1
#define RC_REALLY_LOST 2

extern int32_t rc_values[RADIO_CTL_NB];
extern uint8_t rc_status;
extern int32_t avg_rc_values[RADIO_CTL_NB];
extern uint8_t rc_values_contains_avg_channels;
extern uint8_t time_sime_last_ppm;

static inline void radio_control_init ( void ) {
  rc_status = RC_REALLY_LOST; 
  time_sime_last_ppm = RC_REALLY_LOST_TIME;
}

static inline void radio_control_periodic_task ( void ) {
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
}

static inline bool_t radio_control_ppm_event ( void ) {
  if (ppm_valid) {
    time_sime_last_ppm = 0;
    rc_status = RC_OK;
    NormalizePpm();
    LED_ON(1);
    //  static uint32_t foo;
    //foo++;
    //  if (!(foo%10)) {
    //    PRINT_RADIO_CONTROL();
    //  }
    ppm_valid = FALSE;
    return TRUE;
  }
  return FALSE;
}

#endif /* RADIO_CONTROL_H */
