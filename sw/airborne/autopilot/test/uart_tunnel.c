#include <inttypes.h>
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <math.h>


#include "../uart.h"
#include "../timer.h"

#define UART_PC_PORT    PORTE
#define UART_PC_DDR     DDRE
#define UART_PC_PIN     PINE
#define UART_PC_TX      1
#define UART_PC_RX      0

#define UART_PERPH_PORT PORTD
#define UART_PERPH_DDR  DDRD
#define UART_PERPH_PIN  PIND
#define UART_PERPH_TX   3
#define UART_PERPH_RX   2


void on_uart0_rx(uint8_t c) {
  uart1_transmit(c);
}

void on_uart1_rx(uint8_t c) {
  uart0_transmit(c);
}

ReceiveUart0(on_uart0_rx)
ReceiveUart1(on_uart1_rx)
   
int main( void ) {

  /* setup PC_TX as output */
  UART_PC_DDR |= _BV(UART_PC_TX);
  
  /* setup PC_RX as input, no pullup */
  UART_PC_DDR &= ~_BV(UART_PC_RX);
  UART_PC_PORT &= ~_BV(UART_PC_RX);
  
  /* setup PERPH_TX as output */
  UART_PERPH_DDR |= _BV(UART_PERPH_TX);
  
  /* setup PERPH_RX as input, no pullup */
  UART_PERPH_DDR &= ~_BV(UART_PERPH_RX);
  UART_PERPH_PORT &= ~_BV(UART_PERPH_RX);
  
  while(1)
  {
    if (bit_is_set(UART_PERPH_PIN, UART_PERPH_RX)) {
      sbi(UART_PC_PORT, UART_PC_TX);
    } else {
      cbi(UART_PC_PORT, UART_PC_TX);
    }
    if (bit_is_set(UART_PC_PIN, UART_PC_RX)) {
      sbi(UART_PERPH_PORT, UART_PERPH_TX);
    } else {
      cbi(UART_PERPH_PORT, UART_PERPH_TX);
    }
    
  }
  
  uart0_init();
  uart1_init();

  sei();
  return 0;

}
