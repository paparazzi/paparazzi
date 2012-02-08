/*
 * Paparazzi microcontroller initialisation function
 *
 * Copyright (C) 2010 The Paparazzi team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "mcu.h"

#ifdef PERIPHERALS_AUTO_INIT
#include "mcu_periph/sys_time.h"
#ifdef USE_LED
#include "led.h"
#endif
#if defined RADIO_CONTROL
#if defined RADIO_CONTROL_LINK  || defined RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT
#include "subsystems/radio_control.h"
#endif
#endif
#if defined USE_UART0 || defined USE_UART1  || defined USE_UART2 || defined USE_UART3 || defined USE_UART4  || defined USE_UART5
#include "mcu_periph/uart.h"
#endif
#if defined USE_I2C0  || defined USE_I2C1  || defined USE_I2C2
#include "mcu_periph/i2c.h"
#endif
#if defined USE_ADC
#include "mcu_periph/adc.h"
#endif
#ifdef USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif
#ifdef USE_SPI
#include "mcu_periph/spi.h"
#endif
#ifdef USE_DAC
#include "mcu_periph/dac.h"
#endif
#endif /* PERIPHERALS_AUTO_INIT */

void mcu_init(void) {

  mcu_arch_init();

#ifdef PERIPHERALS_AUTO_INIT
  sys_time_init();
#ifdef USE_LED
  led_init();
#endif
  /* for now this means using spektrum */
#if defined RADIO_CONTROL & defined RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT & defined RADIO_CONTROL_BIND_IMPL_FUNC
  RADIO_CONTROL_BIND_IMPL_FUNC();
#endif
#ifdef USE_UART0
  uart0_init();
#endif
#ifdef USE_UART1
  uart1_init();
#endif
#ifdef USE_UART2
  uart2_init();
#endif
#ifdef USE_UART3
  uart3_init();
#endif
#ifdef USE_UART4
  uart4_init();
#endif
#ifdef USE_UART5
  uart5_init();
#endif
#ifdef USE_I2C0
  i2c0_init();
#endif
#ifdef USE_I2C1
  i2c1_init();
#endif
#ifdef USE_I2C2
  i2c2_init();
#endif
#ifdef USE_ADC
  adc_init();
#endif
#ifdef USE_USB_SERIAL
  VCOM_init();
#endif
#ifdef USE_SPI
  spi_init();
#endif
#ifdef USE_DAC
  dac_init();
#endif
#endif /* PERIPHERALS_AUTO_INIT */

}
