/*
 * Copyright (C) 2010-2012 The Paparazzi team
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

/**
 * @file mcu.c
 * @brief Arch independent mcu ( Micro Controller Unit ) utilities.
 */

#include "mcu.h"
#include "std.h"

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
#if USE_UART0 || USE_UART1 || USE_UART2 || USE_UART3 || USE_UART4 || USE_UART5 || USE_UART6 || USE_UART7 || USE_UART8
#define USING_UART 1
#include "mcu_periph/uart.h"
#endif
#if USE_I2C0 || USE_I2C1 || USE_I2C2 || USE_I2C3
#define USING_I2C 1
#include "mcu_periph/i2c.h"
#endif
#if USE_ADC
#include "mcu_periph/adc.h"
#endif
#if USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif
#ifdef USE_UDP
#include "mcu_periph/udp.h"
#endif
#if USE_SPI
#include "mcu_periph/spi.h"
#endif
#ifdef USE_DAC
#include "mcu_periph/dac.h"
#endif
#endif /* PERIPHERALS_AUTO_INIT */

void WEAK board_init(void)
{
  /* default board init function does nothing... */
}

void mcu_init(void)
{
  /* If we have a board specific init function, call it.
   * Otherwise it will simply call the empty weak function.
   *
   * For example the ARDrone2 has this implemented to prevent stray data of IMU
   * from OEM program still running and also accessing AC sensors
   */
  board_init();

  mcu_arch_init();

  /* First enable the power of the MCU if needed */
#if defined MCU_PWR
  gpio_setup_output(MCU_PWR, MCU_PWR_PIN);
#if defined BTN_ON
  if(!gpio_get(BTN_ON, BTN_ON_PIN))
#endif
  {
  MCU_PWR_ON(MCU_PWR, MCU_PWR_PIN);
  }
#endif

#ifdef PERIPHERALS_AUTO_INIT
  sys_time_init();
#ifdef USE_LED
  led_init();
#endif
  /* First enable power of RC */
#if defined RADIO_CONTROL_POWER
  gpio_setup_output(RADIO_CONTROL_POWER, RADIO_CONTROL_POWER_PIN);
  RADIO_CONTROL_POWER_ON(RADIO_CONTROL_POWER, RADIO_CONTROL_POWER_PIN);
#endif
  /* for now this means using spektrum */
#if defined RADIO_CONTROL & defined RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT & defined RADIO_CONTROL_BIND_IMPL_FUNC & defined SPEKTRUM_BIND_PIN_PORT
  RADIO_CONTROL_BIND_IMPL_FUNC();
#endif
#if USE_UART0
  uart0_init();
#endif
#if USE_UART1
  uart1_init();
#endif
#if USE_UART2
  uart2_init();
#endif
#if USE_UART3
  uart3_init();
#endif
#if USE_UART4
  uart4_init();
#endif
#if USE_UART5
  uart5_init();
#endif
#if USE_UART6
  uart6_init();
#endif
#if USE_UART7
  uart7_init();
#endif
#if USE_UART8
  uart8_init();
#endif
#if USING_UART
  uart_arch_init();
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
#ifdef USE_I2C3
  i2c3_init();
#endif
#if USE_ADC
  adc_init();
#endif
#if USE_USB_SERIAL
  VCOM_init();
#endif

#if USE_SPI
#if SPI_MASTER

#if USE_SPI0
  spi0_init();
#endif
#if USE_SPI1
  spi1_init();
#endif
#if USE_SPI2
  spi2_init();
#endif
#if USE_SPI3
  spi3_init();
#endif
  spi_init_slaves();
#endif // SPI_MASTER

#if SPI_SLAVE
#if USE_SPI0_SLAVE
  spi0_slave_init();
#endif
#if USE_SPI1_SLAVE
  spi1_slave_init();
#endif
#if USE_SPI2_SLAVE
  spi2_slave_init();
#endif
#if USE_SPI3_SLAVE
  spi3_slave_init();
#endif
#endif // SPI_SLAVE

#if SPI_SLAVE_HS
  spi_slave_hs_init();
#endif
#endif // USE_SPI

#ifdef USE_DAC
  dac_init();
#endif

#if USE_UDP0 || USE_UDP1 || USE_UDP2
  udp_arch_init();
#endif

#else
  INFO("PERIPHERALS_AUTO_INIT not enabled! Peripherals (including sys_time) need explicit initialization.")
#endif /* PERIPHERALS_AUTO_INIT */

}



void mcu_event(void)
{
#if USING_I2C
  i2c_event();
#endif

#if USE_USB_SERIAL
  VCOM_event();
#endif
}
