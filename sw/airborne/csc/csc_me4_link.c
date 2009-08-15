#include "csc_me4_link.h"

#include "LPC21xx.h"
#include "armVIC.h"
#include "led.h"


struct CscMe4Link csc_me4_link;

void SPI1_ISR(void) __attribute__((naked));



#define PINSEL1_SCK  (1<<2)
#define PINSEL1_MISO (1<<4)
#define PINSEL1_MOSI (1<<6)
#define PINSEL1_SSEL (1<<8)

#define S1SPCR_CPHA  (0<<3)  /* sample on first edge */
#define S1SPCR_CPOL  (0<<4)  /* clock idles low      */
#define S1SPCR_MSTR  (0<<5)  /* slave mode           */
#define S1SPCR_LSBF  (0<<6)  /* lsb first            */
#define S1SPCR_SPIE  (1<<7)  /* interrupt enable     */

#define S1SPCR_VAL (S1SPCR_CPHA|S1SPCR_CPOL|S1SPCR_MSTR|S1SPCR_LSBF|S1SPCR_SPIE) 
#define S1SPCCR_VAL 0x64


/* S1SPSR bits */
#define ROVR 5
#define WCOL 6
#define SPIF 7

void csc_me4_link_init(void) {
  /* setup pin mux for SPI1 (SCK, MISO, MOSI, SS) */
  PINSEL1 |= PINSEL1_SCK | PINSEL1_MISO | PINSEL1_MOSI | PINSEL1_SSEL;
  /* configure SPI1 */
  S0SPCR  = S1SPCR_VAL;
  S1SPCCR = S1SPCCR_VAL;
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);                       // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);                         // SPI1 interrupt enabled
  _VIC_CNTL(CSC_ME4_LINK_VIC_SLOT) = VIC_ENABLE | VIC_SPI0;
  _VIC_ADDR(CSC_ME4_LINK_VIC_SLOT) = (uint32_t)SPI1_ISR;    // address of the ISR
}




void csc_me4_link_periodic(void) {

}


void SPI1_ISR(void) {
  ISR_ENTRY();

  static uint8_t cnt = 0;
  LED_TOGGLE(2);

  /* transfer complete  */ 
  if ( bit_is_set(S1SPSR, SPIF)) { 
    uint8_t foo __attribute__ ((unused)) = S1SPDR;
    S1SPDR = cnt;
    cnt++;
  }
  
  /* clear_it */
  S1SPINT = 1<<SPI1IF;


 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}

