/*
 * Copyright (C) 2013 Martin Mueller <martinmm@pfump.org>
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

/** \file log_sd_spi.c
 *  \brief host side part of SD card logger through SPI
 *
 */

#include "std.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "mcu_periph/spi.h"

#include "modules/log_sd/log_sd_spi.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

/* ssp input clock 468.75kHz, clock that divided by SCR+1 */
#define SSP_CLOCK 100000

/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : data captured on first clock transition */
#define SSP_CPHA 0x00 << 7  /* clock phase       : SCK idles low */
#define SSP_SCR  0x0F << 8  /* serial clock rate : divide by 16  */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

#define SS_PIN   20
#define SS_IODIR IO0DIR
#define SS_IOSET IO0SET
#define SS_IOCLR IO0CLR

#define ScpSelect()   SetBit(SS_IOCLR,SS_PIN)
#define ScpUnselect() SetBit(SS_IOSET,SS_PIN)

uint8_t log_sd_spi_status;
uint8_t log_sd_spi_run;
uint8_t log_status = 0;

static void log_sd_spi_req(void);
static void SPI1_ISR(void) __attribute__((naked));


void log_sd_spi_init(void) {
  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;

  /* setup SSP */
  SSPCR0 = SSP_DDS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  /* set prescaler for SSP clock PCLK / 16 / 8 = 117kHz */
  SSPCPSR = 8;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR

  /* configure SS pin */
  SetBit(SS_IODIR, SS_PIN); /* pin is output  */
  ScpUnselect();            /* pin idles high */

  log_sd_spi_status = LOG_SD_SPI_UNINIT;
  log_sd_spi_run = 1;
}

static void log_sd_spi_req(void) {
  uint8_t cmd  = 0x23;
  uint8_t data = 0xF0 | log_sd_spi_run;
  uint8_t turb_id=0;
  uint32_t sync_itow=0, cycle_time=0;

  ScpSelect();
  SSPDR = cmd;
  SSPDR = data;
  SpiEnableRti();
  SpiEnable();

  DOWNLINK_SEND_WINDTURBINE_STATUS_(DefaultChannel, DefaultDevice,
    &log_sd_spi_run,
    &turb_id,
    &sync_itow,
    &cycle_time );
}

void log_sd_spi_event( void ) {
}

void log_sd_spi_periodic(void) {
  static unsigned int cntlog = 0;
  if (cpu_time_sec > 1) {
    log_sd_spi_req();
    if (cntlog++ > 5) {
      cntlog = 0;
      DOWNLINK_SEND_BOOZ_DEBUG_FOO(DefaultChannel, DefaultDevice, &log_status);
    }
  }
}

void log_sd_spi_dummy(void) {
}

void log_sd_spi_start(void) {
  log_sd_spi_run = 1;
}

void log_sd_spi_stop(void) {
  log_sd_spi_run = 0;
}

void SPI1_ISR(void) {
  ISR_ENTRY();

  uint8_t foo __attribute__ ((unused));
  foo = SSPDR;
  foo = SSPDR;
 
  if ((foo & 0xFC) == 0x10) log_status = (foo & 0x03);

  ScpUnselect();
  SpiClearRti();
  SpiDisable();

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}
