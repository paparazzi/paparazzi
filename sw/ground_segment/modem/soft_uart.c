#include "soft_uart.h"

#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>

#define FALSE 0
#define TRUE (!FALSE)


volatile uint8_t soft_uart_got_byte = FALSE;
uint8_t soft_uart_byte;
volatile uint8_t soft_uart_error = 0;

#define RX_CLOCKED_DATA_PORT PORTB
#define RX_CLOCKED_DATA_DDR  DDRB
#define RX_CLOCKED_DATA_PIN  PINB
#define RX_CLOCKED_DATA      0


void soft_uart_init(void) {

  /* set CD pin as input, no pullup */
  SOFT_UART_CD_DDR &= ~_BV(SOFT_UART_CD);
  SOFT_UART_CD_PORT &= ~_BV(SOFT_UART_CD);

  /* set DATA pin as input no pullup*/
  RX_CLOCKED_DATA_DDR &= ~_BV(RX_CLOCKED_DATA);
  RX_CLOCKED_DATA_PORT &= ~_BV(RX_CLOCKED_DATA);

  /* setup rx interrupt on failing edge of clock */
  MCUCR = _BV(ISC11);
  /* clear interrupt flag */
  sbi(GIFR, INTF1);
  /* enable interrupt     */
  sbi(GICR, INT1);
}


SIGNAL(SIG_INTERRUPT1) {
  static uint8_t rx_buf_idx = 0;
  static uint8_t rx_buf;

  if (bit_is_clear(SOFT_UART_CD_PIN, SOFT_UART_CD)) {
    rx_buf_idx = 0;
  }
  else {
    if (rx_buf_idx==0) {
      // start bit
      if (bit_is_clear(RX_CLOCKED_DATA_PIN, RX_CLOCKED_DATA)) {
	rx_buf = 0;
	rx_buf_idx++;
      }
    }
    else if (rx_buf_idx < 9) {
      // data bits
      rx_buf >>= 1;
      if (bit_is_set(RX_CLOCKED_DATA_PIN, RX_CLOCKED_DATA))
	rx_buf |= 0x80;
      rx_buf_idx++;
    }
    else {
      // stop bit
      if (bit_is_set(RX_CLOCKED_DATA_PIN, RX_CLOCKED_DATA)) {
	if (soft_uart_got_byte) {
	  soft_uart_error = RX_ERROR_OVERRUN;
	}
	else {
	  soft_uart_byte = rx_buf;
	  soft_uart_got_byte = TRUE;
	}
      }
      else {
	// framing error 
	soft_uart_error = RX_ERROR_FRAMING;
      }
      rx_buf_idx = 0;
    }
  }
}
