#include <inttypes.h>
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <math.h>


#include "../uart.h"
#include "../timer.h"

void on_uart0_rx(uint8_t c) {
  uart1_transmit(c);
}

void on_uart1_rx(uint8_t c) {
  uart0_transmit(c);
}

ReceiveUart0(on_uart0_rx)
ReceiveUart1(on_uart1_rx)
   
int main( void ) {
  uart0_init();
  uart1_init();

  sei();
  return 0;

}
