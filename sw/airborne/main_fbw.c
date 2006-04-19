/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2003-2006 Pascal Brisset, Antoine Drouin
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

#include "main_fbw.h"

#include "init_hw.h"
#include "interrupt_hw.h"

#include "sys_time.h"
#include "commands.h"
#include "actuators.h"
#include "radio_control.h"
#include "fbw_downlink.h"

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


uint8_t fbw_mode;

#include "inter_mcu.h"


#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY 3
// for compatibility
#endif



/********** INIT *************************************************************/
void init_fbw( void ) {
  /** Hardware init */
  hw_init();
#ifdef LED
  led_init();
#endif
#ifdef USE_UART0
  uart0_init_tx();
#if defined IMU_3DMG
  uart0_init_rx();
#else
  Uart0PrintString("FBW Booting $Id$\n");
#endif
#endif

  adc_init();
  adc_buf_channel(ADC_CHANNEL_VSUPPLY, &vsupply_adc_buf, DEFAULT_AV_NB_SAMPLE);
#if defined IMU_3DMG || defined IMU_ANALOG
  imu_init();
#endif
  sys_time_init();

#ifdef ACTUATORS
  actuators_init();
  /* Load the failsafe defaults                     */
  SetCommands(commands_failsafe);
#endif
  
#ifdef RADIO_CONTROL
 ppm_init();
#endif

#ifdef MCU_SPI_LINK
  spi_init();
#endif

  int_enable();

#if IMU_RESET_ON_BOOT
  imu_capture_neutral();
#endif
}


/********** EVENT ************************************************************/
void event_task_fbw( void) {

#ifdef RADIO_CONTROL
  radio_control_event_task();
  if ( rc_status == RC_OK ) {
    if (rc_values_contains_avg_channels) {
      fbw_mode = FBW_MODE_OF_PPRZ(rc_values[RADIO_MODE]);
    }
    if (fbw_mode == FBW_MODE_MANUAL)
      SetCommandsFromRC(commands);
  } else if (fbw_mode == FBW_MODE_MANUAL && rc_status == RC_REALLY_LOST) {
    fbw_mode = FBW_MODE_AUTO;
  }
#endif

#ifdef MCU_SPI_LINK
  if ( !SpiIsSelected() && spi_was_interrupted ) {
    spi_was_interrupted = FALSE;
    spi_reset();
  }
#endif

#ifdef INTER_MCU
  inter_mcu_event_task();
  if (ap_ok && fbw_mode == FBW_MODE_AUTO) {
    SetCommands(from_ap.from_ap.channels);
  }
#endif

#ifdef IMU_3DMG
  if (_3dmg_data_ready) {
    imu_update();
  }
#endif

  if (
#ifdef RADIO_CONTROL
      (fbw_mode == FBW_MODE_MANUAL && rc_status == RC_REALLY_LOST) ||
#endif
#ifdef INTER_MCU
      (fbw_mode == FBW_MODE_AUTO && !ap_ok) ||
#endif
      FALSE
      ) {
    fbw_mode = FBW_MODE_FAILSAFE;
    SetCommands(commands_failsafe);
  }
}


/************* PERIODIC ******************************************************/
void periodic_task_fbw( void ) {

#if defined IMU_ANALOG
  imu_update();
#endif

#if defined IMU_3DMG || defined IMU_ANALOG
  control_rate_run();
  if (rc_status == RC_OK) {
    if (rc_values[RADIO_THROTTLE] < 0.1*MAX_PPRZ) {
      SetCommands(commands_failsafe);
    }
  }
#endif
  
#ifdef RADIO_CONTROL
  radio_control_periodic_task();
#endif

#ifdef INTER_MCU
  inter_mcu_periodic_task();
#endif

#ifdef DOWNLINK
  fbw_downlink_periodic_task();
#endif

  SetActuatorsFromCommands(commands);
}
