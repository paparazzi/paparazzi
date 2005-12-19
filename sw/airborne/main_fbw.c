/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */



#include <inttypes.h>
#include "int.h"

#include "timer_fbw.h"
#include "command.h"
#include "ppm.h"
#include "spi_fbw_hw.h"
#include "spi_fbw.h"
#include "inter_mcu.h"
#include "radio.h"
#include "led.h"


#include "uart_fbw.h"

#ifdef IMU_3DMG 
#include "3dmg.h"
#endif

#if defined  IMU_ANALOG || defined IMU_3DMG
#include "imu.h"
#include "control.h"
#endif

#include "adc_fbw.h"
struct adc_buf vsupply_adc_buf;

uint8_t mode;
static uint8_t time_since_last_ap;
static uint16_t time_since_last_ppm;
bool_t radio_ok, ap_ok, radio_really_lost, failsafe_mode;

static const pprz_t failsafe[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t ppm_cpt, last_ppm_cpt;

#define STALLED_TIME        30  // 500ms with a 60Hz timer
#define REALLY_STALLED_TIME 300 // 5s with a 60Hz timer


/* Prepare data to be sent to mcu0 */
static inline void to_autopilot_from_last_radio (void) {
  uint8_t i;
  for(i = 0; i < RADIO_CTL_NB; i++)
    from_fbw.channels[i] = last_radio[i];
  from_fbw.status = (radio_ok ? _BV(STATUS_RADIO_OK) : 0);
  from_fbw.status |= (radio_really_lost ? _BV(RADIO_REALLY_LOST) : 0);
  from_fbw.status |= (mode == MODE_AUTO ? _BV(STATUS_MODE_AUTO) : 0);
  from_fbw.status |= (failsafe_mode ? _BV(STATUS_MODE_FAILSAFE) : 0);
  if (last_radio_contains_avg_channels) {
    from_fbw.status |= _BV(AVERAGED_CHANNELS_SENT);
    last_radio_contains_avg_channels = FALSE;
  }
  from_fbw.ppm_cpt = last_ppm_cpt;
  from_fbw.vsupply = VoltageOfAdc(vsupply_adc_buf.sum/AV_NB_SAMPLE) * 10;
#if defined IMU_3DMG || defined IMU_ANALOG
  from_fbw.euler_dot[0] = roll_dot;
  from_fbw.euler_dot[1] = pitch_dot;
  from_fbw.euler_dot[2] = yaw_dot;
#endif
#ifdef IMU_3DMG
  from_fbw.euler[0] = roll;
  from_fbw.euler[1] = pitch;
  from_fbw.euler[2] = yaw;
#endif
}

inline void radio_control_task(void) {
  ppm_cpt++;
  radio_ok = TRUE;
  radio_really_lost = FALSE;
  time_since_last_ppm = 0;
  last_radio_from_ppm();
  if (last_radio_contains_avg_channels) {
    mode = MODE_OF_PPRZ(last_radio[RADIO_MODE]);
  }
#if defined IMU_ANALOG && defined RADIO_SWITCH1
  if (last_radio[RADIO_SWITCH1] > MAX_PPRZ/2) {
    imu_capture_neutral();
    CounterLedOn();
  } else {
    CounterLedOff();
  } 
#endif
  if (mode == MODE_MANUAL) {
#if defined IMU_3DMG || defined IMU_ANALOG
    roll_dot_pgain = -100. ; /***  + (float)last_radio[RADIO_GAIN1] * 0.010; ***/
    roll_dot_dgain = 0.; /*** 2.5 - (float)last_radio[RADIO_GAIN2] * 0.00025; ***/
    pitch_dot_pgain = roll_dot_pgain;
    pitch_dot_dgain = roll_dot_dgain;
    control_set_desired(last_radio);
#else
    command_set(last_radio);
#endif  
  }
}

#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY 3
// for compatibility
#endif

void init_fbw( void ) {
  {
    uint8_t foo1 = 25;
    while (foo1--) {
      uint16_t foo2 = 1;
      while (foo2++);
    }
  }
  uart0_init_tx();
#if defined IMU_3DMG
  uart0_init_rx();
#else
  uart0_print_string("FBW Booting $Id$\n");
#endif
  adc_init();
  adc_buf_channel(ADC_CHANNEL_VSUPPLY, &vsupply_adc_buf);
#if defined IMU_3DMG || defined IMU_ANALOG
  CounterLedInit();
  imu_init();
#endif
  timer_init();

  command_init();
  ppm_init();

#ifndef AP
  spi_init();
#endif
/** #else Statically linked with AP: no spi com */

  int_enable();

#if IMU_RESET_ON_BOOT
#warning IMU_RESET_ON_BOOT
  imu_capture_neutral();
#endif
}


void event_task_fbw( void) {
  if( ppm_valid ) {
    ppm_valid = FALSE;
    radio_control_task();
  } else if (mode == MODE_MANUAL && radio_really_lost) {
    mode = MODE_AUTO;
  }

#ifndef AP
  if ( !SpiIsSelected() && spi_was_interrupted ) {
    spi_was_interrupted = FALSE;
    spi_reset();
  }
#endif
/** #else Statically linked with AP: no spi com */

  if (from_ap_receive_valid) {
    time_since_last_ap = 0;
    ap_ok = TRUE;
    if (mode == MODE_AUTO) {
#if defined IMU_ANALOG || defined IMU_3DMG
      control_set_desired(from_ap.channels);
#else
      command_set(from_ap.channels);
#endif
    }
    to_autopilot_from_last_radio();
  }

#ifdef IMU_3DMG
  if (_3dmg_data_ready) {
    imu_update();
  }
#endif

  if (time_since_last_ppm >= STALLED_TIME) {
    radio_ok = FALSE;
  }

  if (time_since_last_ppm >= REALLY_STALLED_TIME) {
    radio_really_lost = TRUE;
  }

  if (time_since_last_ap == STALLED_TIME) {
    ap_ok = FALSE;
  }
  
  failsafe_mode = FALSE;
  if ((mode == MODE_MANUAL && !radio_ok) ||
      (mode == MODE_AUTO && !ap_ok)) {
    failsafe_mode = TRUE;
    command_set(failsafe);
  }
}

void periodic_task_fbw( void ) {
  static uint8_t _1Hz;
  static uint8_t _20Hz;
  _1Hz++;
  _20Hz++;
#if defined IMU_ANALOG
  imu_update();
#endif
#if defined IMU_3DMG || defined IMU_ANALOG
  control_run();
  if (radio_ok) {
    if (last_radio[RADIO_THROTTLE] > 0.1*MAX_PPRZ) {
      command_set(control_commands);
    }
    else {
      command_set(failsafe);
    }
  }
#endif
  if (_1Hz >= 60) {
    _1Hz = 0;
    last_ppm_cpt = ppm_cpt;
    ppm_cpt = 0;
  }
  if (_20Hz >= 3) {
    _20Hz = 0;
#ifndef IMU_3DMG
    /*  	servo_transmit(); */
#endif
  }
  if (time_since_last_ap < STALLED_TIME)
    time_since_last_ap++;
  if (time_since_last_ppm < REALLY_STALLED_TIME)
    time_since_last_ppm++;
}
