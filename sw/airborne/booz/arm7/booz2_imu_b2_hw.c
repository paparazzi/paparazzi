#include "booz2_imu_b2.h"


static void SSP_ISR(void) __attribute__((naked));


/* SSPCR0 settings */
//#define SSP_DDS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_DDS  0x0F << 0  /* data size         : 16 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : data captured on first clock transition */  
#define SSP_CPHA 0x00 << 7  /* clock phase       : SCK idles low */
#define SSP_SCR  0x0F << 8  /* serial clock rate : divide by 16  */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

#define SSPCR0_VAL (SSP_DDS |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR )
#define SSPCR1_VAL (SSP_LBM |  SSP_SSE | SSP_MS | SSP_SOD )

#define SSP_PINSEL1_SCK  (2<<2)
#define SSP_PINSEL1_MISO (2<<4)
#define SSP_PINSEL1_MOSI (2<<6)

void booz2_imu_b2_hw_init(void) {
  
  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= SSP_PINSEL1_SCK  | SSP_PINSEL1_MISO | SSP_PINSEL1_MOSI;
  
  /* setup SSP */
  SSPCR0 = SSPCR0_VAL;;
  SSPCR1 = SSPCR1_VAL;
  SSPCPSR = 0x02;
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( VIC_SPI1 );  /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT( VIC_SPI1 );    /* enable it            */
  _VIC_CNTL(SSP_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_ADDR(SSP_VIC_SLOT) = (uint32_t)SSP_ISR;      /* address of the ISR   */

}


static void SSP_ISR(void) {
 ISR_ENTRY();
 
 switch (booz2_imu_spi_selected) {
   case BOOZ2_SPI_SLAVE_MAX1168:
     {
       Max1168OnSpiInt();
       booz2_imu_spi_selected = BOOZ2_SPI_NONE;
     }
     break;
#ifdef USE_MICROMAG
    case BOOZ2_SPI_SLAVE_MM:
     {
       MmOnSpiIt();
       if (booz2_micromag_status == MM_DATA_AVAILABLE)
         booz2_imu_spi_selected = BOOZ2_SPI_NONE;
     }
     break;
#endif
 }

 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}
