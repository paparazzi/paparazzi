#include "micromag_fw.h"
#include "led.h"


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

#define SSPCR0_VAL (SSP_DDS |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR )
#define SSPCR1_VAL (SSP_LBM |  SSP_SSE | SSP_MS | SSP_SOD )

#define SSP_PINSEL1_SCK  (2<<2)
#define SSP_PINSEL1_MISO (2<<4)
#define SSP_PINSEL1_MOSI (2<<6)

#define SSP_Enable()     SetBit(SSPCR1, SSE);
#define SSP_Disable()    ClearBit(SSPCR1, SSE);
#define SSP_EnableRxi()  SetBit(SSPIMSC, RXIM)
#define SSP_DisableRxi() ClearBit(SSPIMSC, RXIM)
#define SSP_EnableTxi()  SetBit(SSPIMSC, TXIM)
#define SSP_DisableTxi() ClearBit(SSPIMSC, TXIM)
#define SSP_EnableRti()  SetBit(SSPIMSC, RTIM);
#define SSP_DisableRti() ClearBit(SSPIMSC, RTIM);
#define SSP_ClearRti()   SetBit(SSPICR, RTIC);

volatile uint8_t micromag_status;
volatile int16_t micromag_values[MM_NB_AXIS];

static void SSP_ISR(void) __attribute__((naked));

void micromag_periodic( void ) {

  static uint8_t cnt = 0;

  if (micromag_status == MM_IDLE) {
    //    uint8_t * tab = &cnt;
    //    DOWNLINK_SEND_DEBUG(1,tab);
    cnt = 0;
    MmSendReq();
  }
  else if (micromag_status ==  MM_GOT_EOC) {
    MmReadRes();
  }
  else if (micromag_status == MM_WAITING_EOC) {
    cnt++;
    if (cnt > 50) {cnt = 0; micromag_status = MM_IDLE;}
  }
}

static void SSP_ISR(void) {
 ISR_ENTRY();

 MmOnSpiIt();

 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}

void micromag_init_ssp(void) {

  /* setup pins for SSP (SCK, MISO, MOSI, SSEL) */
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

void micromag_init( void ) {

  micromag_hw_init();
  
  uint8_t i;
  for (i=0; i<MM_NB_AXIS; i++)
    micromag_values[i] = 0;
  micromag_status = MM_IDLE;
}

void micromag_reset() {
  micromag_status = MM_IDLE;
}

void micromag_read() {
  if (micromag_status == MM_IDLE) {
    MmSendReq();
  }
  else if (micromag_status ==  MM_GOT_EOC) {
    MmReadRes();
  }
  else if (micromag_status ==  MM_DATA_AVAILABLE) {
    micromag_status == MM_IDLE;
  }
}

