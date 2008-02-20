#include "booz_imu.h"

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

static void SPI1_ISR(void) __attribute__((naked));

void booz_imu_hw_init(void) {

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;
  
  /* setup SSP */
  SSPCR0 = SSP_DDS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = 0x2;
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR 

}


static inline bool_t isr_try_mag(void) {
  switch (micromag_status) {
  case MM_IDLE :
    MmSendReq();
    return TRUE;
  case MM_GOT_EOC:
    MmReadRes();
    return TRUE;
  }
  return FALSE;
}

static inline bool_t isr_try_baro(void) {
#ifndef SCP1000_NO_EINT
  switch (scp1000_status) { 
  case SCP1000_STA_STOPPED:
    Scp1000SendConfig();
    return TRUE;    
  case SCP1000_STA_GOT_EOC:
    Scp1000Read();
    return TRUE;
  }
#else
  if (scp1000_status == SCP1000_STA_STOPPED) {
    Scp1000SendConfig();
    return TRUE;
  }
  else if (scp1000_status == SCP1000_STA_WAIT_EOC && Scp1000DataReady()) {
    scp1000_status = SCP1000_STA_GOT_EOC;
    Scp1000Read();
    return TRUE;
  }
#endif
  return FALSE; 
}

static void SPI1_ISR(void) {
 ISR_ENTRY();

 switch (booz_imu_status) {
 
 case BOOZ_IMU_STA_MEASURING_GYRO:
   Max1167OnSpiInt();
   /* we now have a gyro reading */
   if (isr_try_mag())
     booz_imu_status = BOOZ_IMU_STA_MEASURING_MAG;
   else if (isr_try_baro())
     booz_imu_status = BOOZ_IMU_STA_MEASURING_BARO;
   else
     booz_imu_status = BOOZ_IMU_STA_IDLE;
   break;

 case BOOZ_IMU_STA_MEASURING_MAG:
   MmOnSpiIt();
   if (isr_try_baro())
     booz_imu_status = BOOZ_IMU_STA_MEASURING_BARO;
   else
     booz_imu_status = BOOZ_IMU_STA_IDLE;
   break;
   
 case BOOZ_IMU_STA_MEASURING_BARO:
   Scp1000OnSpiIt();
   booz_imu_status = BOOZ_IMU_STA_IDLE;
   break;
 }

 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}
