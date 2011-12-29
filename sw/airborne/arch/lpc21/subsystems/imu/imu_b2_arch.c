/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

#include "subsystems/imu.h"

int imu_overrun = 0;
volatile uint8_t imu_ssp_status;
static void SSP_ISR(void) __attribute__((naked));

/* SSPCR0 settings */
#define SSP_DDS8  0x07 << 0  /* data size         : 8 bits                    */
#define SSP_DDS16 0x0F << 0  /* data size         : 16 bits                   */
#define SSP_FRF   0x00 << 4  /* frame format      : SPI                       */
#define SSP_CPOL  0x00 << 6  /* clock polarity    : data captured on first clock transition */
#define SSP_CPHA  0x00 << 7  /* clock phase       : SCK idles low             */
#define SSP_SCR   0x0F << 8  /* serial clock rate : divide by 16              */

/* SSPCR1 settings */
#define SSP_LBM   0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE   0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS    0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD   0x00 << 3  /* slave output disable : don't care when master */

#define SSPCR0_VAL8  (SSP_DDS8  |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR )
#define SSPCR0_VAL16 (SSP_DDS16 |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR )
#define SSPCR1_VAL   (SSP_LBM   |  SSP_SSE | SSP_MS   | SSP_SOD )

#define SSP_PINSEL1_SCK  (2<<2)
#define SSP_PINSEL1_MISO (2<<4)
#define SSP_PINSEL1_MOSI (2<<6)


#define ImuSetSSP8bits() { \
    SSPCR0 = SSPCR0_VAL8;      \
}

#define ImuSetSSP16bits() { \
    SSPCR0 = SSPCR0_VAL16;	\
}


void imu_b2_arch_init(void) {

  imu_ssp_status = IMU_SSP_STA_IDLE;

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= SSP_PINSEL1_SCK  | SSP_PINSEL1_MISO | SSP_PINSEL1_MOSI;

  /* setup SSP */
  SSPCR0 = SSPCR0_VAL16;
  SSPCR1 = SSPCR1_VAL;
  SSPCPSR = 0x02;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( VIC_SPI1 );             /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT( VIC_SPI1 );               /* enable it            */
  _VIC_CNTL(SSP_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_ADDR(SSP_VIC_SLOT) = (uint32_t)SSP_ISR;      /* address of the ISR   */

}


void imu_periodic(void) {
  // check ssp idle
  if (imu_ssp_status != IMU_SSP_STA_IDLE)
  {
    imu_overrun++;
    return; //, DEBUG_IMU, IMU_ERR_OVERUN);
  }

  // setup 16 bits
  ImuSetSSP16bits();
  // read adc
  imu_ssp_status = IMU_SSP_STA_BUSY_MAX1168;
  max1168_read();
#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_AMI601
  RunOnceEvery(10, { ami601_read(); });
#endif
#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC58XX
  RunOnceEvery(5,Hmc58xxPeriodic());
#endif

}



#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100

static void SSP_ISR(void) {
  ISR_ENTRY();

  switch (imu_ssp_status) {
  case IMU_SSP_STA_BUSY_MAX1168:
    Max1168OnSpiInt();
    if (ms2100_status == MS2100_IDLE || ms2100_status == MS2100_GOT_EOC) {
      ImuSetSSP8bits();
      if (ms2100_status == MS2100_IDLE) {
        Ms2001SendReq();
      }
      else { /* MS2100_GOT_EOC */
        Ms2001ReadRes();
      }
      imu_ssp_status = IMU_SSP_STA_BUSY_MS2100;
    }
    else {
      imu_ssp_status = IMU_SSP_STA_IDLE;
    }
    break;
  case IMU_SSP_STA_BUSY_MS2100:
    Ms2001OnSpiInt();
    if (ms2100_status == MS2100_IDLE) {
      Ms2001SendReq();
      imu_ssp_status = IMU_SSP_STA_BUSY_MS2100;
    }
    else
      imu_ssp_status = IMU_SSP_STA_IDLE;
    break;

   // default:
   // spurious interrupt
   // FIXME LED_ON(1);
  }

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

#else //no IMU_B2_MAG_MS2100

static void SSP_ISR(void) {
  ISR_ENTRY();

  switch (imu_ssp_status) {
  case IMU_SSP_STA_BUSY_MAX1168:
    Max1168OnSpiInt();
    imu_ssp_status = IMU_SSP_STA_IDLE;
    break;

    // default:
    // spurious interrupt
    // FIXME LED_ON(1);
  }

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

#endif //no IMU_B2_MAG_MS2100
