#include "booz_link_mcu.h"

#ifdef BOOZ_FILTER_MCU


#include "armVIC.h"

volatile uint8_t spi0_data_available;
volatile uint8_t spi0_idx_buf;
uint8_t* spi0_buffer_output = (uint8_t*)&inter_mcu_state;
uint8_t* spi0_buffer_input = (uint8_t*)&booz_link_mcu_state_unused;



void SPI0_ISR(void) __attribute__((naked));

#define Spi0InitBuf() {						      \
    spi0_idx_buf = 0;						      \
    S0SPDR = spi0_buffer_output[0];				      \
  }

#define Spi0OneByte() {							\
    /* FIXME : do something usefull with the status register reading */ \
    uint8_t foo __attribute__((unused)) = S0SPSR;			\
    spi0_buffer_input[spi0_idx_buf] = S0SPDR;				\
    spi0_idx_buf++;							\
    if (spi0_idx_buf < sizeof(inter_mcu_state)) {			\
      S0SPDR = spi0_buffer_output[spi0_idx_buf];			\
    }									\
    else {								\
      spi0_data_available = TRUE;					\
      BoozLinkMcuSetUnavailable();						\
    }									\
  } 



/*  */
#define S0SPCR_SPIE  7  /* SPI enable */

void booz_link_mcu_hw_init ( void ) {
  /* init SPI0 */
  /* setup pins for sck, miso, mosi, SSEL */
  PINSEL0 |= 1<<8 | 1<<10| 1<<12 | 1<< 14;
  /* setup P1_16 to P1_25 as GPIO */
  PINSEL2 &= ~(1<<3);
  /* P1_24 (DRDY) is output */
  SetBit(IO1DIR, SPI0_DRDY);
  /* DRDY idles high */
  BoozLinkMcuSetUnavailable();
  /* configure SPI : 8 bits CPOL=0 CPHA=0 MSB_first slave */
  S0SPCR = _BV(3);
  /* setup SPI clock rate */
  S0SPCCR = 0x20;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI0);   // SPI0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI0);     // SPI0 interrupt enabled
  VICVectCntl1 = VIC_ENABLE | VIC_SPI0;
  VICVectAddr1 = (uint32_t)SPI0_ISR;    // address of the ISR

  /* clear pending interrupt */
  SetBit(S0SPINT, SPI0IF);
  /* enable SPI interrupt */
  SetBit(S0SPCR, S0SPCR_SPIE);

}


void SPI0_ISR(void) {
  ISR_ENTRY();
  Spi0OneByte();
  /* clear the interrupt */
  S0SPINT = _BV(SPI0IF); 
  /* clear this interrupt from the VIC */
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}



#endif /* BOOZ_FILTER_MCU */

#ifdef BOOZ_CONTROLLER_MCU  
/* IMU connected to SSP ( aka SPI1 ) */

#include "LPC21xx.h"
#include "interrupt_hw.h" 

/* DRDY connected pin to P0.16 EINT0 */ 
#define LINK_IMU_DRDY_PINSEL PINSEL1
#define LINK_IMU_DRDY_PINSEL_VAL 0x01
#define LINK_IMU_DRDY_PINSEL_BIT 0
#define LINK_IMU_DRDY_EINT 0

static void EXTINT0_ISR(void) __attribute__((naked));

void booz_link_mcu_hw_init ( void ) {


  /* configure DRDY pin */
  LINK_IMU_DRDY_PINSEL |= LINK_IMU_DRDY_PINSEL_VAL << LINK_IMU_DRDY_PINSEL_BIT;
  SetBit(EXTMODE, LINK_IMU_DRDY_EINT);   /* EINT is edge trigered */
  ClearBit(EXTPOLAR,LINK_IMU_DRDY_EINT); /* EINT is trigered on falling edge */
  SetBit(EXTINT,LINK_IMU_DRDY_EINT);     /* clear pending EINT */
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( VIC_EINT0 );  /* select EINT0 as IRQ source */
  VICIntEnable = VIC_BIT( VIC_EINT0 );    /* enable it */
  VICVectCntl9 = VIC_ENABLE | VIC_EINT0;
  VICVectAddr9 = (uint32_t)EXTINT0_ISR;   // address of the ISR 

  spi_buffer_input = (uint8_t*)&inter_mcu_state;
  spi_buffer_output = (uint8_t*)&booz_link_mcu_state_unused;
  spi_buffer_length = sizeof(inter_mcu_state);


}


void EXTINT0_ISR(void) {
  ISR_ENTRY();

  SpiSelectSlave0();
  SpiStart();

  /* clear EINT2 */
  SetBit(EXTINT,LINK_IMU_DRDY_EINT); /* clear EINT0 */
  
  VICVectAddr = 0x00000000;    /* clear this interrupt from the VIC */
  ISR_EXIT();
}

#endif /* BOOZ_CONTROLLER_MCU */
