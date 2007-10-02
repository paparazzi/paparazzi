#include "dc_mc_link.h"

#include <avr/interrupt.h>
#include <avr/io.h>

/*
  Slave address 
  front = 0x52
  back  = 0x54
  right = 0x56
  left  = 0x58
*/
#define DC_MC_LINK_TWI_ADDR 0x52
#define DC_MC_LINK_TIMEOUT 60

volatile uint8_t dc_mc_link_event;
uint16_t dc_mc_link_command;
uint8_t dc_mc_link_timeout;

#define DC_MC_LINK_TWI_RX_BUF_LEN 16
uint8_t dc_mc_link_twi_rx_buf[DC_MC_LINK_TWI_RX_BUF_LEN];
uint8_t dc_mc_link_twi_rx_buf_idx;


void dc_mc_link_init(void) {
  dc_mc_link_event = FALSE;
  dc_mc_link_command = 0;
  dc_mc_link_timeout = DC_MC_LINK_TIMEOUT;

  /* setup slave addr */
  TWAR = 0x52;
  /* clear and enable interrupt - enable peripheral and ack bit */
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);

}

void dc_mc_link_periodic(void) {
  if (dc_mc_link_timeout >= DC_MC_LINK_TIMEOUT)
    dc_mc_link_command = 0;
  else
    dc_mc_link_timeout++;
}

#define TWI_BUS_ERR_1   0x00
#define TWI_BUS_ERR_2   0xF8

// Status Slave RX Mode
#define SR_SLA_ACK      0x60
#define SR_LOST_ACK     0x68
#define SR_GEN_CALL_ACK 0x70
#define GEN_LOST_ACK    0x78
#define SR_PREV_ACK     0x80
#define SR_PREV_NACK    0x88
#define GEN_PREV_ACK    0x90
#define GEN_PREV_NACK   0x98
#define STOP_CONDITION  0xA0
#define REPEATED_START  0xA0

// Status Slave TX mode
#define SW_SLA_ACK      0xA8
#define SW_LOST_ACK     0xB0
#define SW_DATA_ACK     0xB8
#define SW_DATA_NACK    0xC0
#define SW_LAST_ACK     0xC8

#include "led.h"

ISR (TWI_vect) {  
  switch (TWSR & 0xF8) {
    case SR_SLA_ACK:  
      //      LED_OFF(1);
      dc_mc_link_twi_rx_buf_idx = 0;
      TWCR |= _BV(TWINT) | _BV(TWEA);
      break;
    case SR_PREV_ACK:
      dc_mc_link_twi_rx_buf[dc_mc_link_twi_rx_buf_idx] = TWDR;
      dc_mc_link_twi_rx_buf_idx++;
      if (dc_mc_link_twi_rx_buf_idx <= 2)
	TWCR |= _BV(TWINT) | _BV(TWEA);
      else
	TWCR |= _BV(TWINT);
      break;
    case STOP_CONDITION:
      TWCR |= _BV(TWINT);
      dc_mc_link_timeout = 0;
      //      LED_ON(1);
      dc_mc_link_event = TRUE;
      break;
    case SW_SLA_ACK:  
    case SW_DATA_ACK:
      TWCR |= (1<<TWINT);
      break;
    case TWI_BUS_ERR_2:
    case TWI_BUS_ERR_1:
      TWCR |=(1<<TWSTO) | (1<<TWINT); 
      TWCR =(1<<TWEA) | (1<<TWINT) | (1<<TWEN) | (1<<TWIE); // TWI Reset
      break;
    
      //    default:
      //      LED_TOGGLE(1);
  }

}
