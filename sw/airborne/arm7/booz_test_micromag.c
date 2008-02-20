#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "LPC21xx.h"
#include "6dof.h"
#include "micromag.h"
#include "spi_hw.h"


static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

static void SPI1_ISR(void) __attribute__((naked));
static void my_spi_init(void);

int16_t mag_raw[AXIS_NB];

uint32_t t0, t1, diff;

int main( void ) {
  main_init();
  while (1) {
    if (sys_time_periodic())
      main_periodic();
    main_event();
  }
  return 0;
}


static inline void main_init(void) {
  hw_init();
  sys_time_init();
  uart1_init_tx();

  my_spi_init();
  micromag_init();

  int_enable();
}


static inline void main_periodic(void) {
  //  DOWNLINK_SEND_BOOT(&cpu_time_sec );
  switch (micromag_status) {
  case MM_IDLE :
    t0 = T0TC;
    MmSendReq();
    break;
  case MM_GOT_EOC:
    MmReadRes();
    break;
  }
}


static inline void main_event(void) {
  if (micromag_status == MM_DATA_AVAILABLE) {
      micromag_status = MM_IDLE;
      mag_raw[AXIS_X] = micromag_values[0];
      mag_raw[AXIS_Y] = micromag_values[1];
      mag_raw[AXIS_Z] = micromag_values[2];
      DOWNLINK_SEND_IMU_MAG_RAW(&mag_raw[AXIS_X], &mag_raw[AXIS_Y], &mag_raw[AXIS_Z]);
      t1 = T0TC;
      diff = t1 - t0;
      DOWNLINK_SEND_TIME(&diff);
  }
}


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


static void my_spi_init(void) {

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;
  
  /* setup SSP */
  SSPCR0 = SSP_DDS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  //  SSPCPSR = 0x20;
  SSPCPSR = 0x2;
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR 

}

static void SPI1_ISR(void) {
 ISR_ENTRY();

 MmOnSpiIt();

 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}
