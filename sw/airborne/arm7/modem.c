#include "modem.h"

#include "types.h"
#include "LPC21xx.h"
#include "armVIC.h"
#include "config.h"

#define TX_BUF_SIZE 255

uint8_t           modem_nb_ovrn;
uint8_t           tx_head;
volatile uint8_t  tx_tail;
uint8_t           tx_buf[ TX_BUF_SIZE ];
uint8_t ck_a, ck_b;
uint8_t    tx_byte;
uint8_t    tx_byte_idx;

#define T1_PCLK_DIV 3

#define SAMPLE_PERIOD 0xFFF
#define SAMPLES_PER_PERIOD 10
#define NB_STATE 2
#define NB_PHASE 2

static const uint16_t modem_sample[NB_STATE][NB_PHASE][SAMPLES_PER_PERIOD] = 
  {
    {{512, 812, 997, 997, 812, 512,  211,  26,  26, 211},
     {512 ,211 , 26 , 26, 211 ,511,  812, 997, 997, 812}},
    {{512, 669, 812, 925, 997, 1023, 997, 925, 812, 669},
     {512 ,354 ,211 , 98,  26,    1, 26 ,  98, 211, 354}}
  };

static uint8_t modem_sample_idx = 0;
static uint8_t modem_phase = 0;

void modem_init ( void ) {
  /* turn on DAC pins */
  PINSEL1 &= 1 << 19;
  PINSEL1 |= ~(1 << 18);
  /* reset & disable timer 1   */
  T1TCR = TCR_RESET;
  /* set the prescale divider  */
  T1PR = T1_PCLK_DIV - 1;
 /* select TIMER1 as IRQ       */
  VICIntSelect &= ~VIC_BIT(VIC_TIMER1);
  /* enable TIMER1 interrupt   */
  VICIntEnable = VIC_BIT(VIC_TIMER1);
  /* on slot vic slot 5        */
  VICVectCntl5 = VIC_ENABLE | VIC_TIMER1;
  /* address of the ISR        */
  VICVectAddr5 = (uint32_t)TIMER1_ISR;
  /* trigger initial match in a long time from now */
  T1MR0 = 0XFFFF;
  /* enable interrupt on match register 0 */
  T1MCR |= TMCR_MR0_I;
  /* enable timer 1 */
  T1TCR = TCR_ENABLE; 
}

void modem_put_one_byte( uint8_t _byte) {
  tx_buf[tx_head] = _byte;
  tx_head++;
  if (tx_head >= TX_BUF_SIZE) tx_head = 0;
}

static inline uint8_t get_next_bit( void ) {
  uint8_t ret;
  /*  start bit         */
  if (tx_byte_idx == 0)
    ret = 0;
  /*  data bits         */
  else if (tx_byte_idx < 9) {
    ret = tx_byte & 0x01;
    tx_byte >>= 1;
  }
  /* stop_bit           */
  else {
    ret = 1;
  }
  tx_byte_idx++;
  if (tx_byte_idx >= 10) {
    /*  if we have nothing left to transmit */
    if( tx_head == tx_tail ) {
      /* hack to stay with data = 1            */
      tx_byte_idx--;
    } else {
      /* else load next byte                  */
      tx_byte = tx_buf[tx_tail];	       
      tx_byte_idx = 0;				
      tx_tail++;				
      if( tx_tail >= TX_BUF_SIZE )
	tx_tail = 0;
    }
  }
  return ret;
}


void TIMER1_ISR ( void ) {
  ISR_ENTRY();
  uint8_t modem_bit =  get_next_bit();
  
  DACR = modem_sample[modem_bit][modem_phase][modem_sample_idx] << 6;
  modem_sample_idx++;
  if (modem_sample_idx == SAMPLES_PER_PERIOD) {
    modem_sample_idx = 0;
    if (modem_bit)
      modem_phase = !modem_phase;
  }

  /* trigger next match */
  T1MR0 += SAMPLE_PERIOD;
  /* clear interrupt */
  T1IR = TIR_MR0I;
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}



