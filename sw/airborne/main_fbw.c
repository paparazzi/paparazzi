/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2003-2005 Pascal Brisset, Antoine Drouin
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

#include "init_hw.h"
#include "main_fbw.h"
#include "int.h"
#include "sys_time.h"
#include "actuators.h"
#include "commands.h"
#include "ppm.h"

#include "led.h"
#include "uart.h"
#include "print.h"

#ifdef MCU_SPI_LINK
#include "spi_fbw_hw.h"
#include "spi_fbw.h"
#endif

#ifdef IMU_3DMG 
#include "3dmg.h"
#endif

#if defined IMU_ANALOG || defined IMU_3DMG
#include "imu.h"
#include "control_grz.h"
#endif

#include "adc.h"
struct adc_buf vsupply_adc_buf;


static uint8_t mode;
static uint16_t time_since_last_ppm;
static bool_t radio_ok, radio_really_lost, failsafe_mode;

static pprz_t rc_values[ PPM_NB_PULSES ];
static pprz_t avg_rc_values[ PPM_NB_PULSES ];
static bool_t rc_values_contains_avg_channels = FALSE;

static uint8_t ppm_cpt, last_ppm_cpt;

#define STALLED_TIME        30  // 500ms with a 60Hz timer
#define REALLY_STALLED_TIME 300 // 5s with a 60Hz timer


#include "inter_mcu.h"
#include "inter_mcu_fbw.h"


#define RC_AVG_PERIOD 8
/* Copy from the ppm receiving buffer to the buffer sent to mcu0 */
static void rc_values_from_ppm( void ) {
  NormalizePpm();
}

static inline void radio_control_task(void) {
  ppm_cpt++;
  radio_ok = TRUE;
  radio_really_lost = FALSE;
  time_since_last_ppm = 0;
  rc_values_from_ppm();
  if (rc_values_contains_avg_channels) {
    mode = FBW_MODE_OF_PPRZ(rc_values[RADIO_MODE]);
  }
#if defined IMU_ANALOG && defined RADIO_SWITCH1
  if (rc_values[RADIO_SWITCH1] > MAX_PPRZ/2) {
    imu_capture_neutral();
    CounterLedOn();
  } else {
    CounterLedOff();
  } 
#endif
  if (mode == FBW_MODE_MANUAL) {
#if defined IMU_3DMG || defined IMU_ANALOG
    roll_dot_pgain = -100. ; /***  + (float)rc_values[RADIO_GAIN1] * 0.010; ***/
    roll_dot_dgain = 0.; /*** 2.5 - (float)rc_values[RADIO_GAIN2] * 0.00025; ***/
    pitch_dot_pgain = roll_dot_pgain;
    pitch_dot_dgain = roll_dot_dgain;
#endif

    SetCommandsFromRC(commands);
  }
}

#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY 3
// for compatibility
#endif

void init_fbw( void ) {
  hw_init();
  uart0_init_tx();
#if defined IMU_3DMG
  uart0_init_rx();
#else
  Uart0PrintString("FBW Booting $Id$\n");
#endif
  adc_init();
  adc_buf_channel(ADC_CHANNEL_VSUPPLY, &vsupply_adc_buf, DEFAULT_AV_NB_SAMPLE);
#if defined IMU_3DMG || defined IMU_ANALOG
  imu_init();
#endif
  sys_time_init();

  actuators_init();

  /* Load the failsafe defaults                     */
  SetCommands(commands_failsafe);
  
  ppm_init();

#ifdef MCU_SPI_LINK
  spi_init();
#endif

  int_enable();

#if IMU_RESET_ON_BOOT
  imu_capture_neutral();
#endif
}


void event_task_fbw( void) {
  if( ppm_valid ) {
    ppm_valid = FALSE;
    radio_control_task();
  } else if (mode == FBW_MODE_MANUAL && radio_really_lost) {
    mode = FBW_MODE_AUTO;
  }

#ifdef MCU_SPI_LINK
  if ( !SpiIsSelected() && spi_was_interrupted ) {
    spi_was_interrupted = FALSE;
    spi_reset();
  }
#endif

#ifdef INTER_MCU
  inter_mcu_event_task();
#endif

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

  failsafe_mode = FALSE;
  if ((mode == FBW_MODE_MANUAL && !radio_ok)
#ifdef INTER_MCU
      || (mode == FBW_MODE_AUTO && !ap_ok)
#endif
      ) {
    failsafe_mode = TRUE;
    SetCommands(commands_failsafe);
  }
}

void periodic_task_fbw( void ) {
  static uint8_t _1Hz;
  _1Hz++;

#if defined IMU_ANALOG
  imu_update();
#endif
#if defined IMU_3DMG || defined IMU_ANALOG
  control_rate_run();
  if (radio_ok) {
    if (rc_values[RADIO_THROTTLE] < 0.1*MAX_PPRZ) {
      SetCommands(failsafe);
    }
  }
#endif

  if (_1Hz >= 60) {
    _1Hz = 0;
    last_ppm_cpt = ppm_cpt;
    ppm_cpt = 0;
  }

#ifdef INTER_MCU
  inter_mcu_periodic_task();
#endif

  if (time_since_last_ppm < REALLY_STALLED_TIME)
    time_since_last_ppm++;

  SetActuatorsFromCommands(commands);
}
