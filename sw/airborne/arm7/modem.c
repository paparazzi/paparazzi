#include "modem.h"

#include "types.h"
#include "LPC21xx.h"
#include "armVIC.h"
#include "config.h"

uint8_t           modem_nb_ovrn;
uint8_t           tx_head;
volatile uint8_t  tx_tail;
uint8_t           tx_buf[ TX_BUF_SIZE ];
uint8_t ck_a, ck_b;
uint8_t    tx_byte;
uint8_t    tx_bit_idx;

#define T1_PCLK_DIV 3

#define SAMPLES_PER_PERIOD 4
#define SAMPLE_PERIOD (PCLK/4762/SAMPLES_PER_PERIOD/T1_PCLK_DIV)

#define NB_STATE 2
#define NB_PHASE 2

static const uint16_t modem_sample[NB_STATE][NB_PHASE][SAMPLES_PER_PERIOD] = 
  {
    {{512, 1023, 512, 1},
     {512 ,1 , 512 , 1023}},
    {{512, 874, 1023, 874},
     {512 ,150 ,1 , 150}}
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
  /* on slot vic slot 1        */
  VICVectCntl1 = VIC_ENABLE | VIC_TIMER1;
  /* address of the ISR        */
  VICVectAddr1 = (uint32_t)TIMER1_ISR;
  /* trigger initial match in a long time from now */
  T1MR0 = SAMPLE_PERIOD;
  /* enable interrupt on match register 0 */
  T1MCR |= TMCR_MR0_I | TMCR_MR0_R;
  /* enable timer 1 */
  T1TCR = TCR_ENABLE; 
}

static inline uint8_t get_next_bit( void ) {
  uint8_t ret;
  /*  start bit         */
  if (tx_bit_idx == 0)
    ret = 0;
  /*  data bits         */
  else if (tx_bit_idx < 9) {
    ret = tx_byte & 0x01;
    tx_byte >>= 1;
  }
  /* stop_bit           */
  else {
    ret = 1;
  }
  tx_bit_idx++;
  
  if (tx_bit_idx >= 10) {
    /*  if we have nothing left to transmit */
    if( tx_head == tx_tail ) {
      /* hack to stay with data = 1            */
      tx_bit_idx--;
    } else {
      /* else load next byte                  */
      tx_byte = tx_buf[tx_tail];	       
      tx_bit_idx = 0;				
      tx_tail++;				
            if( tx_tail >= TX_BUF_SIZE )
	      tx_tail = 0; 
    }
  }
  return ret;
}

void TIMER1_ISR ( void ) {
  ISR_ENTRY();
  //  IO1CLR = LED_2_BIT;
  static uint8_t modem_bit;
  
  DACR = modem_sample[modem_bit][modem_phase][modem_sample_idx] << 6;
  modem_sample_idx++;
  if (modem_sample_idx == SAMPLES_PER_PERIOD) {
    modem_sample_idx = 0;
    modem_phase ^= modem_bit;
    modem_bit =  get_next_bit();
  }

  /* clear interrupt */
  T1IR = TIR_MR0I;
  VICVectAddr = 0x00000000;
  //  IO1SET = LED_2_BIT;  
  ISR_EXIT();
}



