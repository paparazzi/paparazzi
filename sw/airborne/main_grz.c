#include "pprz_transport.h"
#include "datalink.h"
#include "uart.h"
#include "interrupt_hw.h"
#include "periodic.h"
#include "ap_downlink.h"
#include "flight_plan.h"
#include "control_grz.h"
#include "adc.h"
#include "radio_control.h"
#include "autopilot.h"
#include "gps.h"
#include "estimator.h"


uint8_t pprz_mode = PPRZ_MODE_MANUAL;
uint8_t vertical_mode = 0;
uint8_t lateral_mode = 0;
uint8_t horizontal_mode = 0;
uint8_t rc_settings_mode = 0;
uint8_t mcu1_status = 0;
uint8_t ir_estim_mode = 0;


/** From nav.c */
int32_t nav_utm_east0; 
int32_t nav_utm_north0;


struct adc_buf rangemeter_adc_buf;
uint16_t rangemeter; /* cm */


#define NB_RANGES 30
#define FLYING_MIN_CM 30
#define STILL_MAX_CM 18
#define SWITCH_PEDIOD_MIN 30 /* 60Hz */


/** 60Hz */
void periodic_task_ap( void ) {

  PORTD |= 1 << 5;

  static uint8_t _10Hz   = 0;
  _10Hz++;
  if (_10Hz>=6) _10Hz = 0;

  if (!_10Hz)
    PeriodicSendAp();

  static uint16_t last_rangemeters[NB_RANGES];
  static uint8_t last_idx;
  static uint16_t sum_rangemeters = 0;
  // ClearBit(ADCSRA, ADIE);
  rangemeter = RangeCmOfAdc(rangemeter_adc_buf.sum/rangemeter_adc_buf.av_nb_sample);
  // SetBit(ADCSRA, ADIE);
  ctl_grz_z = rangemeter / 100.; /* cm -> m */
  last_idx++;
  if (last_idx >= NB_RANGES) last_idx = 0;
  float last_avg = (float)sum_rangemeters / NB_RANGES;
  sum_rangemeters += rangemeter - last_rangemeters[last_idx];
  last_rangemeters[last_idx] = rangemeter;
  float avg = (float)sum_rangemeters / NB_RANGES;
  ctl_grz_z_dot = ((avg - last_avg) * 61. / 100.);


  /** Flying ? */
  static uint8_t flying_cpt;
  if (!flying) {
    if (rangemeter > FLYING_MIN_CM) {
      flying_cpt++;
      if (flying_cpt > SWITCH_PEDIOD_MIN) {
	flying = TRUE;
	flying_cpt = 0;
      }
    } else
      flying_cpt = 0;
  } else if (rangemeter < STILL_MAX_CM) {
    flying_cpt++;
    if (flying_cpt > SWITCH_PEDIOD_MIN) {
      flying = FALSE;
      flying_cpt = 0;
    }
  } else
    flying_cpt = 0;


  pprz_mode = PPRZ_MODE_OF_PULSE(rc_values[RADIO_MODE], FIXME);

  if (pprz_mode == PPRZ_MODE_AUTO2) {
    ctl_grz_set_setpoints_auto2();
  } 
  else if (pprz_mode == PPRZ_MODE_AUTO1) {
    ctl_grz_alt_run();
    ctl_grz_set_setpoints_auto1();
  }

  if (pprz_mode >= PPRZ_MODE_AUTO2) {
    if (!_10Hz)
      ctl_grz_horiz_speed_run();
  }

  if (pprz_mode >= PPRZ_MODE_AUTO1) {
    ctl_grz_speed_run();
    ctl_grz_attitude_run();
  }

  PORTD &= ~ (1 << 5);
}

void event_task_ap( void ) {

  /***** Datalink *******/
  if (PprzBuffer()) {
    ReadPprzBuffer();
    if (pprz_msg_received) {
      pprz_parse_payload();
      pprz_msg_received = FALSE;

      if (dl_msg_available) {
	dl_parse_msg();
	dl_msg_available = FALSE;
      }
    }
  }

  /* parse and use GPS messages */
  if (GpsBuffer()) {
    ReadGpsBuffer();
  }
  if (gps_msg_received) {
    parse_gps_msg();
    gps_msg_received = FALSE;
    if (gps_pos_available) {
      use_gps_pos();
      gps_pos_available = FALSE;
    }
  }

  
}


#include <avr/io.h>
void init_ap( void ) {
  OSCCAL=0xB1;

  uart1_init_rx();
  uart0_init_rx();

  /** adc_init done in fbw */
  adc_buf_channel(ADC_CHANNEL_RANGEMETER, &rangemeter_adc_buf, DEFAULT_AV_NB_SAMPLE);

  gps_init ();

  int_enable();

  /** Debug */
  SetBit(DDRD, 5);
}

