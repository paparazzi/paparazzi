#include "traces.h"

void uart1_print_hex( uint8_t c) {
  const uint8_t hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', 
                            '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
  uint8_t high = (c & 0xF0)>>4;
  uint8_t low  = c & 0x0F;
  uart1_transmit(hex[high]);
  uart1_transmit(hex[low]);
}

void uart1_print_hex_16( uint16_t c) {
  uint8_t high = (uint8_t)(c>>8);
  uint8_t low  = (uint8_t)(c);
  uart1_print_hex(high);
  uart1_print_hex(low);
}


