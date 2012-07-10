/*
 * $Id$
 *
 * Copyright (C) 20010 Antoine Drouin <poinix@gmail.com>
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
 */

#include "subsystems/imu.h"

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/nvic.h>

#define IMU_SSP_STA_IDLE           0
#define IMU_SSP_STA_BUSY_MAX1168   1
#define IMU_SSP_STA_BUSY_MS2100    2

volatile uint8_t imu_ssp_status;

void dma1_channel4_isr(void);
void spi2_isr(void);

void imu_b2_arch_init(void) {

  /* Enable SPI2 Periph clock -------------------------------------------------*/
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI2EN);
  /* Enable SPI_2 DMA clock ---------------------------------------------------*/
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);
  /* Enable PORTB GPIO clock --------------------------------------------------*/
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN);
  /* Configure GPIOs: SCK, MISO and MOSI  -------------------------------------*/
  gpio_set_mode(GPIO_BANK_SPI2_SCK, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI2_SCK |
	                                        GPIO_SPI2_MISO |
	                                        GPIO_SPI2_MOSI);
  /* Enable DMA1 channel4 IRQ Channel */
  nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
  nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
  /* Enable SPI2 IRQ Channel */
  nvic_set_priority(NVIC_SPI2_IRQ, 1);
  nvic_enable_irq(NVIC_SPI2_IRQ);

  imu_ssp_status = IMU_SSP_STA_IDLE;
}

void imu_periodic(void) {
  // check ssp idle
  // ASSERT((imu_status == IMU_STA_IDLE), DEBUG_IMU, IMU_ERR_OVERUN);
  imu_ssp_status = IMU_SSP_STA_BUSY_MAX1168;
  Max1168ConfigureSPI();
  spi_enable(SPI2);
  max1168_read();
#if IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC5843
  hmc5843_periodic();
#endif
}

/* used for spi2 */
void dma1_channel4_isr(void) {
  switch (imu_ssp_status) {
  case IMU_SSP_STA_BUSY_MAX1168:
    Max1168OnDmaIrq();
    spi_disable(SPI2);
#if IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
    if (ms2100_status == MS2100_IDLE) {
      Ms2100SendReq();
      imu_ssp_status = IMU_SSP_STA_BUSY_MS2100;
    }
    else if (ms2100_status == MS2100_WAITING_EOC && Ms2100HasEOC()) {
      Ms2100ReadRes();
      imu_ssp_status = IMU_SSP_STA_BUSY_MS2100;
    }
    else
#endif
      imu_ssp_status = IMU_SSP_STA_IDLE;
    break;
  case IMU_SSP_STA_BUSY_MS2100:
#if IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
    Ms2100OnDmaIrq();
#endif
    break;
  default:
    // POST_ERROR(DEBUG_IMU, IMU_ERR_SUPRIOUS_DMA1_C4_IRQ);
    imu_ssp_status = IMU_SSP_STA_IDLE;
  }
}


void spi2_isr(void) {
#if IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
  Ms2100OnSpiIrq();
#endif
}
