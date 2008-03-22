#include "std.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"
#include "led.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"

#include "micromag.h"

static inline void tl_main_init( void );
static inline void tl_main_periodic_task( void );
static inline void tl_main_event_task( void );

static void test_spi_init(void);

int main( void ) {
  tl_main_init();
  while(1) {
    if (sys_time_periodic())
      tl_main_periodic_task();
    tl_main_event_task();
  }
  return 0;
}

static inline void tl_main_init( void ) {
  hw_init();
  led_init();
  sys_time_init();
  uart0_init_tx();

  
  test_spi_init();
  micromag_init();

  int_enable();
}

static inline void tl_main_periodic_task( void ) {
  DOWNLINK_SEND_BOOT(&cpu_time_sec);
  RunOnceEvery(3,{micromag_read();});
}


static inline void tl_main_event_task( void ) {
  if (micromag_status == MM_DATA_AVAILABLE) {
    micromag_status = MM_IDLE;
    DOWNLINK_SEND_IMU_MAG_RAW(&micromag_values[0], &micromag_values[1], &micromag_values[2]); 
  }
}



static void SPI1_ISR(void) __attribute__((naked));


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


static void test_spi_init(void) {
  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;
  
  /* setup SSP */
  SSPCR0 = SSP_DDS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = 0x20;
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR 

}


void SPI1_ISR(void) {
  ISR_ENTRY();
  MicromagOnSpiInt();
  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}
