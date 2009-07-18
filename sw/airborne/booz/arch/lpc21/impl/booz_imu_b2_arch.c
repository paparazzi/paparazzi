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

#include "booz_imu.h"

volatile uint8_t booz_imu_ssp_status;
static void SSP_ISR(void) __attribute__((naked));
#if 0
static inline bool_t isr_try_mag(void);
#endif

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


#define BoozImuSetSSP8bits() { \
    SSPCR0 = SSPCR0_VAL8;      \
}

#define BoozImuSetSSP16bits() { \
    SSPCR0 = SSPCR0_VAL16;	\
}


void booz_imu_b2_arch_init(void) {
  
  booz_imu_ssp_status = BOOZ_IMU_SSP_STA_IDLE;

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


void booz_imu_periodic(void) {
  // check ssp idle
  // ASSERT((booz_imu_status == BOOZ_IMU_STA_IDLE), DEBUG_IMU, IMU_ERR_OVERUN);
  
  // setup 16 bits
  BoozImuSetSSP16bits();
  // read adc
  booz_imu_ssp_status = BOOZ_IMU_SSP_STA_BUSY_MAX1168;
  booz_max1168_read();
  
}



#include "led.h"

#if 0


static inline bool_t isr_try_mag(void) {
  switch (micromag_status) {
  case MM_IDLE :
    Booz2ImuSetSSP8bits();
    MmSendReq();
    return TRUE;
  case MM_GOT_EOC:
    Booz2ImuSetSSP8bits();
    MmReadRes();
    return TRUE;
  }
  return FALSE;
}

static void SSP_ISR(void) {
 ISR_ENTRY();
 
 switch (booz2_imu_ssp_status) {
 case BOOZ2_IMU_SSP_STA_BUSY_MAX1168:
   Max1168OnSpiInt();
   if (isr_try_mag())
     booz2_imu_ssp_status = BOOZ2_IMU_SSP_STA_BUSY_MS2100;
   else
     booz2_imu_ssp_status = BOOZ2_IMU_SSP_STA_IDLE;
   break;
 case BOOZ2_IMU_SSP_STA_BUSY_MS2100:
   MmOnSpiIt();
   booz2_imu_ssp_status = BOOZ2_IMU_SSP_STA_IDLE;
   break;
 default:
   // spurious interrupt
   LED_ON(1);
 }
 
 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}
#endif


static void SSP_ISR(void) {
 ISR_ENTRY();

 switch (booz_imu_ssp_status) {
 case BOOZ_IMU_SSP_STA_BUSY_MAX1168:
   Max1168OnSpiInt();
#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2001 
  if (micromag_status == MM_IDLE || micromag_status == MM_GOT_EOC) {
     BoozImuSetSSP8bits();
     if (micromag_status == MM_IDLE) {
       MmSendReq();
     }
     else { /* MM_GOT_EOC */
       MmReadRes();
     }
     booz_imu_ssp_status = BOOZ_IMU_SSP_STA_BUSY_MS2100;
   }
   else {
#endif
     booz_imu_ssp_status = BOOZ_IMU_SSP_STA_IDLE;
#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2001 
   }
#endif
  break;
#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2001 
 case BOOZ_IMU_SSP_STA_BUSY_MS2100:
   MmOnSpiIt();
   if (micromag_status == MM_IDLE) {
    MmSendReq();
    booz_imu_ssp_status = BOOZ_IMU_SSP_STA_BUSY_MS2100;
   }
   else
     booz_imu_ssp_status = BOOZ_IMU_SSP_STA_IDLE;
   break;
#endif
   // default:
   // spurious interrupt
   // FIXME LED_ON(1);
 }
 
 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}
